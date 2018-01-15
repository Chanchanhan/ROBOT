#include <iostream>
#include <glog/logging.h>

#include "DataPaser.h"
int main(int argc, char* argv[]) {

  google::InitGoogleLogging(argv[0]);
  if (argc < 2) {
        LOG(ERROR) << "Not using  ROBOTConfig.yaml \n";
        return -1;
  }
  OcvYamlConfig ocvYamlConfig(argv[1]);
  auto dataPaser = std::make_unique<DataPaser>(ocvYamlConfig);
  dataPaser->doTraking();
  return 0;
}