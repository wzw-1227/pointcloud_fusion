#include <ehpr_bridge/matching_io.h>
#include <iostream>
#include <chrono>
int main(int argc,char** argv) {
  bool chronological=false,prime=false,ingestion=false;for(int i=4;i<argc;++i){std::string arg=argv[i];
    if(arg=="--chronological")chronological=true;else if(arg=="--prime")prime=true;
    else if(arg=="--ingestion-order"){ingestion=true;chronological=true;}else{std::cerr<<"Unknown option: "<<arg<<'\n';return 1;}}
  if(argc<4){std::cerr<<"Usage: stage3_match_cli STAGE2_DIRECTORY CONFIG OUTPUT_ROOT [--chronological] [--prime] [--ingestion-order]\n";return 1;}
  std::unique_ptr<ehpr_bridge::MatchWriter> writer;
  try {
    auto config=ehpr_bridge::loadMatcherConfig(argv[2]);ehpr_bridge::Matcher matcher(config);
    writer=std::make_unique<ehpr_bridge::MatchWriter>(argv[3],argv[2],"replay");
    matcher.setArchiveDirectory((writer->directory()/"history").string());
    auto inputs=ehpr_bridge::loadReplay(argv[1],chronological,ingestion);
    if(prime)matcher.prime(inputs);
    else for(auto input:inputs){const auto started=std::chrono::steady_clock::now();auto event=matcher.process(input);
      event.diagnostics["matching_processing_wall_s"]=std::chrono::duration<double>(std::chrono::steady_clock::now()-started).count();
      writer->write(event,input,matcher);
      std::cout<<input->identity()<<" -> "<<event.status<<": "<<event.reason<<'\n';}
    if(prime){while(true){auto started=std::chrono::steady_clock::now();auto task=matcher.pollHistory();if(!task)break;
      task->first.diagnostics["matching_processing_wall_s"]=std::chrono::duration<double>(std::chrono::steady_clock::now()-started).count();writer->write(task->first,task->second,matcher);}}
    writer->checkpoint(matcher);
    writer->finish("stopped");
    std::cout<<"RESULT_DIR="<<writer->directory().string()<<'\n';
    return matcher.counters().count("accepted_matches")?0:2;
  } catch(const std::exception& e) {
    std::cerr<<"Stage 3 failure: "<<e.what()<<'\n';
    if(writer)try{writer->finish("failed",e.what());}catch(...){}
    return 1;
  }
}
