// Lightweight OSM 0.6 XML parser: nodes/ways → classified map polylines in
// KITTI world frame.

#include "cam_loc/map/osm_xml_parser.h"

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <unordered_map>
#include <vector>

namespace cam_loc::map {

namespace {

using TagMap = std::unordered_map<std::string, std::string>;

struct OsmNode {
  double lat = 0;
  double lon = 0;
};

struct OsmWay {
  uint64_t id = 0;
  std::vector<uint64_t> refs;
  TagMap tags;
};

std::string Trim(const std::string& s) {
  size_t b = 0;
  while (b < s.size() && (std::isspace(static_cast<unsigned char>(s[b])) != 0))
    ++b;
  size_t e = s.size();
  while (e > b && (std::isspace(static_cast<unsigned char>(s[e - 1])) != 0))
    --e;
  return s.substr(b, e - b);
}

bool ParseDoubleAttr(const std::string& tag, const std::string& key,
                     double& out) {
  const std::string needle = key + "=\"";
  const size_t pos = tag.find(needle);
  if (pos == std::string::npos) return false;
  const size_t start = pos + needle.size();
  const size_t end = tag.find('"', start);
  if (end == std::string::npos) return false;
  // strtod rather than std::stod: the latter reports a malformed attribute by
  // throwing, and end_ptr also lets a partial parse ("12abc") be rejected.
  const std::string text = tag.substr(start, end - start);
  errno = 0;
  char* end_ptr = nullptr;
  const double value = std::strtod(text.c_str(), &end_ptr);
  if (errno != 0 || end_ptr != text.c_str() + text.size()) return false;
  out = value;
  return true;
}

bool ParseUint64Attr(const std::string& tag, const std::string& key,
                     uint64_t& out) {
  const std::string needle = key + "=\"";
  const size_t pos = tag.find(needle);
  if (pos == std::string::npos) return false;
  const size_t start = pos + needle.size();
  const size_t end = tag.find('"', start);
  if (end == std::string::npos) return false;
  const std::string text = tag.substr(start, end - start);
  if (text.empty() || text[0] == '-') return false;
  errno = 0;
  char* end_ptr = nullptr;
  const unsigned long long value = std::strtoull(text.c_str(), &end_ptr, 10);
  if (errno != 0 || end_ptr != text.c_str() + text.size()) return false;
  out = static_cast<uint64_t>(value);
  return true;
}

bool ParseTagKv(const std::string& line, std::string& k, std::string& v) {
  const std::string kneedle = "k=\"";
  const size_t kpos = line.find(kneedle);
  if (kpos == std::string::npos) return false;
  const size_t kstart = kpos + kneedle.size();
  const size_t kend = line.find('"', kstart);
  if (kend == std::string::npos) return false;
  k = line.substr(kstart, kend - kstart);

  const std::string vneedle = "v=\"";
  const size_t vpos = line.find(vneedle);
  if (vpos == std::string::npos) return false;
  const size_t vstart = vpos + vneedle.size();
  const size_t vend = line.find('"', vstart);
  if (vend == std::string::npos) return false;
  v = line.substr(vstart, vend - vstart);
  return true;
}

// The tag tests below all ask one of two questions -- is this key present, and
// does it carry this value -- and spelling them out reached the map twice for
// the second: count() to test, at() to read. These do it in one lookup and read
// as the question, which is what keeps the classification tables below
// scannable as tables.
bool HasTag(const TagMap& tags, const std::string& key) {
  return tags.find(key) != tags.end();
}

bool TagEquals(const TagMap& tags, const std::string& key,
               const std::string& value) {
  const auto it = tags.find(key);
  return it != tags.end() && it->second == value;
}

// Keep drivable/barrier geometry; skip closed areas and pedestrian-only ways
// later.
bool IsMapWay(const TagMap& tags) {
  if (TagEquals(tags, "area", "yes")) return false;
  if (HasTag(tags, "highway")) return true;
  if (HasTag(tags, "barrier")) return true;
  if (TagEquals(tags, "man_made", "kerb")) return true;
  if (HasTag(tags, "railway")) return true;
  return false;
}

// Map OSM highway/barrier tags to lane solid/dashed/edge types used by
// matching.
kitti::PolylineType ClassifyWay(const TagMap& tags) {
  if (HasTag(tags, "barrier")) return kitti::PolylineType::kRoadEdge;
  if (TagEquals(tags, "man_made", "kerb")) {
    return kitti::PolylineType::kRoadEdge;
  }
  if (HasTag(tags, "highway")) {
    const std::string& hw = tags.at("highway");
    if (hw == "footway" || hw == "path" || hw == "steps" ||
        hw == "pedestrian") {
      return kitti::PolylineType::kUnknown;
    }
    if (hw == "cycleway") return kitti::PolylineType::kLaneDashed;
    if (TagEquals(tags, "lanes", "1")) return kitti::PolylineType::kLaneSolid;
    if (hw == "motorway_link" || hw == "trunk_link")
      return kitti::PolylineType::kLaneDashed;
    return kitti::PolylineType::kLaneSolid;
  }
  return kitti::PolylineType::kUnknown;
}

Status ParseOsmXmlImpl(const std::string& xml_text, const MapGeoref& georef,
                       kitti::MapChunk& out, OsmBounds* out_bounds) {
  if (!georef.IsValid()) return Status::kInvalidArgument;

  // Single-pass line scan: collect nodes, ways (refs + tags), optional bounds.
  std::unordered_map<uint64_t, OsmNode> nodes;
  std::vector<OsmWay> ways;
  OsmBounds bounds;
  OsmWay* current_way = nullptr;

  std::istringstream stream(xml_text);
  std::string line;
  while (std::getline(stream, line)) {
    line = Trim(line);
    if (line.empty()) continue;

    if (line.rfind("<bounds", 0) == 0) {
      ParseDoubleAttr(line, "minlat", bounds.min_lat);
      ParseDoubleAttr(line, "minlon", bounds.min_lon);
      ParseDoubleAttr(line, "maxlat", bounds.max_lat);
      ParseDoubleAttr(line, "maxlon", bounds.max_lon);
      bounds.valid = true;
      continue;
    }

    if (line.rfind("<node", 0) == 0) {
      uint64_t id = 0;
      double lat = 0;
      double lon = 0;
      if (!ParseUint64Attr(line, "id", id) ||
          !ParseDoubleAttr(line, "lat", lat) ||
          !ParseDoubleAttr(line, "lon", lon)) {
        continue;
      }
      nodes[id] = OsmNode{lat, lon};
      continue;
    }

    if (line.rfind("<way", 0) == 0) {
      OsmWay way;
      ParseUint64Attr(line, "id", way.id);
      ways.push_back(std::move(way));
      current_way = &ways.back();
      continue;
    }

    if (current_way != nullptr) {
      if (line.rfind("<nd", 0) == 0) {
        uint64_t ref = 0;
        if (ParseUint64Attr(line, "ref", ref)) {
          current_way->refs.push_back(ref);
        }
        continue;
      }
      if (line.rfind("<tag", 0) == 0) {
        std::string k;
        std::string v;
        if (ParseTagKv(line, k, v)) {
          current_way->tags[k] = v;
        }
        continue;
      }
      if (line.rfind("</way>", 0) == 0) {
        current_way = nullptr;
      }
    }
  }

  // Resolve way node refs through georef and emit lane/edge polylines.
  out.polylines.clear();
  for (const auto& way : ways) {
    if (!IsMapWay(way.tags) || way.refs.size() < 2) continue;
    const kitti::PolylineType type = ClassifyWay(way.tags);
    if (type == kitti::PolylineType::kUnknown) continue;

    kitti::MapPolyline3D pl;
    pl.id = way.id;
    pl.type = type;
    for (uint64_t ref : way.refs) {
      const auto it = nodes.find(ref);
      if (it == nodes.end()) continue;
      pl.points.push_back(georef.Wgs84ToWorld(it->second.lat, it->second.lon));
    }
    if (pl.points.size() >= 2) {
      out.polylines.push_back(std::move(pl));
    }
  }

  if (out_bounds != nullptr) {
    *out_bounds = bounds;
  }
  return out.polylines.empty() ? Status::kInvalidArgument : Status::kOk;
}

}  // namespace

Status ParseOsmXml(const std::string& xml_text, const MapGeoref& georef,
                   kitti::MapChunk& out, OsmBounds* out_bounds) {
  return ParseOsmXmlImpl(xml_text, georef, out, out_bounds);
}

Status ParseOsmXmlFile(const std::string& path, const MapGeoref& georef,
                       kitti::MapChunk& out, OsmBounds* out_bounds) {
  std::ifstream in(path);
  if (!in.is_open()) return Status::kIoError;
  std::string xml((std::istreambuf_iterator<char>(in)),
                  std::istreambuf_iterator<char>());
  return ParseOsmXml(xml, georef, out, out_bounds);
}

}  // namespace cam_loc::map
