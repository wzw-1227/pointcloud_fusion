#pragma once
#include <ehpr_bridge/matcher.h>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <filesystem>
namespace ehpr_bridge {
MatcherConfig loadMatcherConfig(const std::string& path);
std::vector<SubmapPtr> loadReplay(const std::filesystem::path& directory, bool chronological=false, bool ingestion_order=false);
YAML::Node inputMetadata(const SubmapInput& input);
YAML::Node matchMetadata(const MatchResult& match);
void saveYamlAtomic(const std::filesystem::path& path,const YAML::Node& data);
class MatchWriter {
 public:
  MatchWriter(const std::string& root,const std::string& config,const std::string& mode);
  std::string write(const MatchEvent& event,SubmapPtr input,const Matcher& matcher);
  void finish(const std::string& status,const std::string& error="");
  void checkpoint(const Matcher& matcher);
  const std::filesystem::path& directory() const { return directory_; }
  YAML::Node summary() const { return summary_; }
 private:
  std::filesystem::path directory_;
  YAML::Node summary_;
  std::deque<YAML::Node> recent_events_;
  std::ofstream journal_;
  int match_id_=0,event_id_=0;
};
}
