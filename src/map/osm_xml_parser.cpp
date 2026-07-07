// Lightweight OSM 0.6 XML parser: nodes/ways → classified map polylines in KITTI world frame.

#include <cam_loc/map/osm_xml_parser.hpp>

#include <algorithm>
#include <cctype>
#include <fstream>
#include <sstream>
#include <unordered_map>
#include <vector>

namespace cam_loc::map {

namespace {

struct OsmNode {
  double lat = 0;
  double lon = 0;
};

struct OsmWay {
  uint64_t id = 0;
  std::vector<uint64_t> refs;
  std::unordered_map<std::string, std::string> tags;
};

std::string trim(const std::string& s) {
  size_t b = 0;
  while (b < s.size() && std::isspace(static_cast<unsigned char>(s[b]))) ++b;
  size_t e = s.size();
  while (e > b && std::isspace(static_cast<unsigned char>(s[e - 1]))) --e;
  return s.substr(b, e - b);
}

bool parseDoubleAttr(const std::string& tag, const std::string& key, double& out) {
  const std::string needle = key + "=\"";
  const size_t pos = tag.find(needle);
  if (pos == std::string::npos) return false;
  const size_t start = pos + needle.size();
  const size_t end = tag.find('"', start);
  if (end == std::string::npos) return false;
  try {
    out = std::stod(tag.substr(start, end - start));
    return true;
  } catch (...) {
    return false;
  }
}

bool parseUint64Attr(const std::string& tag, const std::string& key, uint64_t& out) {
  const std::string needle = key + "=\"";
  const size_t pos = tag.find(needle);
  if (pos == std::string::npos) return false;
  const size_t start = pos + needle.size();
  const size_t end = tag.find('"', start);
  if (end == std::string::npos) return false;
  try {
    out = static_cast<uint64_t>(std::stoull(tag.substr(start, end - start)));
    return true;
  } catch (...) {
    return false;
  }
}

bool parseTagKv(const std::string& line, std::string& k, std::string& v) {
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

// Keep drivable/barrier geometry; skip closed areas and pedestrian-only ways later.
bool isMapWay(const std::unordered_map<std::string, std::string>& tags) {
  if (tags.count("area") && tags.at("area") == "yes") return false;
  if (tags.count("highway")) return true;
  if (tags.count("barrier")) return true;
  if (tags.count("man_made") && tags.at("man_made") == "kerb") return true;
  if (tags.count("railway")) return true;
  return false;
}

// Map OSM highway/barrier tags to lane solid/dashed/edge types used by matching.
kitti::PolylineType classifyWay(const std::unordered_map<std::string, std::string>& tags) {
  if (tags.count("barrier")) return kitti::PolylineType::kRoadEdge;
  if (tags.count("man_made") && tags.at("man_made") == "kerb") {
    return kitti::PolylineType::kRoadEdge;
  }
  if (tags.count("highway")) {
    const std::string& hw = tags.at("highway");
    if (hw == "footway" || hw == "path" || hw == "steps" || hw == "pedestrian") {
      return kitti::PolylineType::kUnknown;
    }
    if (hw == "cycleway") return kitti::PolylineType::kLaneDashed;
    if (tags.count("lanes") && tags.at("lanes") == "1") return kitti::PolylineType::kLaneSolid;
    if (hw == "motorway_link" || hw == "trunk_link") return kitti::PolylineType::kLaneDashed;
    return kitti::PolylineType::kLaneSolid;
  }
  return kitti::PolylineType::kUnknown;
}

Status parseOsmXmlImpl(const std::string& xml_text, const MapGeoref& georef,
                       kitti::MapChunk& out, OsmBounds* out_bounds) {
  if (!georef.isValid()) return Status::kInvalidArgument;

  // Single-pass line scan: collect nodes, ways (refs + tags), optional bounds.
  std::unordered_map<uint64_t, OsmNode> nodes;
  std::vector<OsmWay> ways;
  OsmBounds bounds;
  OsmWay* current_way = nullptr;

  std::istringstream stream(xml_text);
  std::string line;
  while (std::getline(stream, line)) {
    line = trim(line);
    if (line.empty()) continue;

    if (line.rfind("<bounds", 0) == 0) {
      parseDoubleAttr(line, "minlat", bounds.min_lat);
      parseDoubleAttr(line, "minlon", bounds.min_lon);
      parseDoubleAttr(line, "maxlat", bounds.max_lat);
      parseDoubleAttr(line, "maxlon", bounds.max_lon);
      bounds.valid = true;
      continue;
    }

    if (line.rfind("<node", 0) == 0) {
      uint64_t id = 0;
      double lat = 0;
      double lon = 0;
      if (!parseUint64Attr(line, "id", id) || !parseDoubleAttr(line, "lat", lat) ||
          !parseDoubleAttr(line, "lon", lon)) {
        continue;
      }
      nodes[id] = OsmNode{lat, lon};
      continue;
    }

    if (line.rfind("<way", 0) == 0) {
      OsmWay way;
      parseUint64Attr(line, "id", way.id);
      ways.push_back(std::move(way));
      current_way = &ways.back();
      continue;
    }

    if (current_way != nullptr) {
      if (line.rfind("<nd", 0) == 0) {
        uint64_t ref = 0;
        if (parseUint64Attr(line, "ref", ref)) {
          current_way->refs.push_back(ref);
        }
        continue;
      }
      if (line.rfind("<tag", 0) == 0) {
        std::string k;
        std::string v;
        if (parseTagKv(line, k, v)) {
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
    if (!isMapWay(way.tags) || way.refs.size() < 2) continue;
    const kitti::PolylineType type = classifyWay(way.tags);
    if (type == kitti::PolylineType::kUnknown) continue;

    kitti::MapPolyline3D pl;
    pl.id = way.id;
    pl.type = type;
    for (uint64_t ref : way.refs) {
      const auto it = nodes.find(ref);
      if (it == nodes.end()) continue;
      pl.points.push_back(georef.wgs84ToWorld(it->second.lat, it->second.lon));
    }
    if (pl.points.size() >= 2) {
      out.polylines.push_back(std::move(pl));
    }
  }

  if (out_bounds) {
    *out_bounds = bounds;
  }
  return out.polylines.empty() ? Status::kInvalidArgument : Status::kOk;
}

}  // namespace

Status parseOsmXml(const std::string& xml_text, const MapGeoref& georef, kitti::MapChunk& out,
                   OsmBounds* out_bounds) {
  return parseOsmXmlImpl(xml_text, georef, out, out_bounds);
}

Status parseOsmXmlFile(const std::string& path, const MapGeoref& georef, kitti::MapChunk& out,
                       OsmBounds* out_bounds) {
  std::ifstream in(path);
  if (!in.is_open()) return Status::kIoError;
  std::string xml((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
  return parseOsmXml(xml, georef, out, out_bounds);
}

}  // namespace cam_loc::map
