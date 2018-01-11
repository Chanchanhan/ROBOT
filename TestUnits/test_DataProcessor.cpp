#include <iostream>
#include <glog/logging.h>

#include "dataPaser.h"
#include "OcvYamlConfig.h"
int main(int argc, char* argv[]) {
      
  google::InitGoogleLogging(argv[0]);  
  if (argc < 2) {
        LOG(ERROR) << "Not using  ROBOTConfig.yaml \n";
        return -1;      
  }
  OcvYamlConfig ocvYamlConfig(argv[1]);
  auto dataPaser = std::make_unique<DataPaser>(argc,argv,ocvYamlConfig); 
  
  return 0;
}