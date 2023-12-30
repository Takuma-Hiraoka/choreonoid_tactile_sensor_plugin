#include <boost/program_options.hpp>
#include <iostream>

#include <cnoid/YAMLReader>
#include <cnoid/YAMLWriter>
#include <cnoid/EigenTypes>
class TactileSensor
{
 public:
  std::string linkName;
  cnoid::Vector3 position; // リンク座標系でどこに取り付けられているか
  cnoid::VectorXd rot; // リンク座標系でセンサの姿勢．zがリンク内側方向
};

bool loadConfig(cnoid::Mapping* topNode, std::vector<TactileSensor>& sensorConfig);
bool writeConfig(cnoid::YAMLWriter* writer, std::vector<TactileSensor> sensorConfig);

int main(int argc, char *argv[]){

  boost::program_options::options_description opt("Options");
  opt.add_options()
    ("help,h", "read tactile sensor distribution(.yaml) and write each tactile sensor configuration(.yaml). config_writer -i [file path] -o [file path]")
    ("input,i", boost::program_options::value<std::string>(),"input file path")
    ("output,o", boost::program_options::value<std::string>(),"output file path");
  boost::program_options::variables_map argmap;
  boost::program_options::store(boost::program_options::parse_command_line(argc, argv, opt), argmap );
  boost::program_options::notify( argmap );

  if( argmap.count( "help" ) ){
    std::cerr << opt << std::endl;
    return 1;
  }
  std::vector<TactileSensor> sensorConfig;
  std::string inputFilePath = argmap["input"].as<std::string>();
  cnoid::YAMLReader reader;
  cnoid::MappingPtr node;
  try {
    node = reader.loadDocument(inputFilePath)->toMapping();
  } catch(const cnoid::ValueNode::Exception& ex) {
    std::cerr << "\e[0;31m" << "[config_writer] cannot find " << inputFilePath <<  "\e[0m" << std::endl;
  }

  if (!loadConfig(node, sensorConfig)) std::cerr << "\e[0;31m" << "[config_writer] cannot load config file" << "\e[0m" << std::endl;

  std::string outputFilePath = argmap["output"].as<std::string>();
  cnoid::YAMLWriter writer(outputFilePath);

  writeConfig(&writer, sensorConfig);
  return 0;
}

bool loadConfig(cnoid::Mapping* topNode, std::vector<TactileSensor>& sensorConfig) {
  auto& tactileSensorList = *topNode->findListing("tactile_sensor");
  if (!tactileSensorList.isValid()) return false;
  for (int i=0; i< tactileSensorList.size(); i++) {
    cnoid::Mapping* info = tactileSensorList[i].toMapping();
    // link
    std::string linkName;
    info->extract("link", linkName);
    // type
    std::string type;
    info->extract("type", type);
    if (type == "rectangle") {
      auto point_ = info->extract("point");
      auto& pointTmp = *point_->toListing();
      cnoid::Vector3 point = cnoid::Vector3(pointTmp[0].toDouble(), pointTmp[1].toDouble(), pointTmp[2].toDouble());
      auto direction1_ = info->extract("direction1");
      auto& direction1Tmp = *direction1_->toListing();
      cnoid::Vector3 direction1 = cnoid::Vector3(direction1Tmp[0].toDouble(), direction1Tmp[1].toDouble(), direction1Tmp[2].toDouble());
      auto direction2_ = info->extract("direction2");
      auto& direction2Tmp = *direction2_->toListing();
      cnoid::Vector3 direction2 = cnoid::Vector3(direction2Tmp[0].toDouble(), direction2Tmp[1].toDouble(), direction2Tmp[2].toDouble());
      int num_dir1 = info->extract("num_dir1")->toInt();
      int num_dir2 = info->extract("num_dir2")->toInt();
      auto rot_ = info->extract("rot");
      auto& rotTmp = *rot_->toListing();
      cnoid::VectorXd rot = cnoid::VectorXd::Zero(9);
      rot << rotTmp[0].toDouble(), rotTmp[1].toDouble(), rotTmp[2].toDouble(),
             rotTmp[3].toDouble(), rotTmp[4].toDouble(), rotTmp[5].toDouble(),
             rotTmp[6].toDouble(), rotTmp[7].toDouble(), rotTmp[8].toDouble();
      for (int j=0; j < num_dir1; j++) {
        for (int k=0; k < num_dir2; k++) {
          TactileSensor sensor;
          sensor.linkName = linkName;
          sensor.position = point + direction1 * j / num_dir1 + direction2 * k / num_dir2;
          sensor.rot = rot;
          sensorConfig.push_back(sensor);
        }
      }
    }
  }
  return true;
}

bool writeConfig(cnoid::YAMLWriter* writer, std::vector<TactileSensor> sensorConfig)
{
  writer->startMapping();
  writer->putKey("tactile_sensor");
  writer->startListing();
  for (int i=0; i<sensorConfig.size(); i++) {
    writer->startMapping();
    writer->startListing();
    {
      writer->putKeyValue("link",sensorConfig[i].linkName);
      writer->putKey("translation");
      writer->startFlowStyleListing();
      for (int j=0; j<sensorConfig[i].position.size(); j++) writer->putScalar(sensorConfig[i].position[j]);
      writer->endListing();
      writer->putKey("rotation");
      writer->startFlowStyleListing();
      for (int j=0; j<sensorConfig[i].rot.size(); j++) writer->putScalar(sensorConfig[i].rot[j]);
      writer->endListing();
    }
    writer->endListing();
    writer->endMapping();
  }
  writer->endListing();
  writer->endMapping();
  return true;
};
