#include <boost/program_options.hpp>
#include <iostream>

#include <cnoid/YAMLReader>
#include <cnoid/YAMLWriter>
#include <cnoid/EigenTypes>
#include <cnoid/EigenUtil>
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
    } else if (type == "cylinder") {
      auto origin_ = info->extract("origin");
      auto& originTmp = *origin_->toListing();
      cnoid::Vector3 origin = cnoid::Vector3(originTmp[0].toDouble(), originTmp[1].toDouble(), originTmp[2].toDouble());
      double init_angle = info->extract("init_angle")->toDouble();
      auto direction_x_ = info->extract("direction_x");
      auto& direction_xTmp = *direction_x_->toListing();
      cnoid::Vector3 direction_x = cnoid::Vector3(direction_xTmp[0].toDouble(), direction_xTmp[1].toDouble(), direction_xTmp[2].toDouble());
      auto direction_y_ = info->extract("direction_y");
      auto& direction_yTmp = *direction_y_->toListing();
      cnoid::Vector3 direction_y = cnoid::Vector3(direction_yTmp[0].toDouble(), direction_yTmp[1].toDouble(), direction_yTmp[2].toDouble());
      double radius = (direction_x).norm();
      auto rot_ = info->extract("rot");
      auto& rotTmp = *rot_->toListing();
      cnoid::Matrix3 rot;
      rot << rotTmp[0].toDouble(), rotTmp[1].toDouble(), rotTmp[2].toDouble(),
             rotTmp[3].toDouble(), rotTmp[4].toDouble(), rotTmp[5].toDouble(),
             rotTmp[6].toDouble(), rotTmp[7].toDouble(), rotTmp[8].toDouble();
      auto direction_cylinder_ = info->extract("direction_cylinder");
      auto& direction_cylinderTmp = *direction_cylinder_->toListing();
      cnoid::Vector3 direction_cylinder = cnoid::Vector3(direction_cylinderTmp[0].toDouble(), direction_cylinderTmp[1].toDouble(), direction_cylinderTmp[2].toDouble());
      std::string is_direction_angle_same;
      info->extract("is_direction_angle_same", is_direction_angle_same);
      double angle_flag = (is_direction_angle_same == "true") ? 1.0 : -1.0;
      std::string is_direction_height_same;
      info->extract("is_direction_height_same", is_direction_height_same);
      double height_flag = (is_direction_height_same == "true") ? 1.0 : -1.0;
      double distance_angle = info->extract("distance_angle")->toDouble();
      double distance_height = info->extract("distance_height")->toDouble();
      int num_angle = info->extract("num_angle")->toInt();
      int num_height = info->extract("num_height")->toInt();
      std::string is_angle_first;
      info->extract("is_angle_first", is_angle_first);
      // TODO 回転座標のx.yも合わせる
      if (is_angle_first == "true") {
	for (int height=0; height < num_height; height++) {
	  for (int angle=0; angle < num_angle; angle++) {
	    TactileSensor sensor;
	    sensor.linkName = linkName;
	    sensor.position = origin + cos(init_angle + angle_flag * distance_angle * angle) * direction_x + sin(init_angle + angle_flag * distance_angle * angle) * direction_y + direction_cylinder * height_flag * height * distance_height;
	    sensor.rot = cnoid::VectorXd::Zero(9);
	    cnoid::Matrix3 rotation = rot * cnoid::rotFromRpy(0,0,init_angle + angle_flag * distance_angle * angle) * cnoid::rotFromRpy(0,cnoid::PI / 2, 0.0);
	    for (int a=0; a<3; a++) {
	      for (int b=0; b<3; b++) {
		sensor.rot[a*3+b] = rotation(a,b);
	      }
	    }
	    sensorConfig.push_back(sensor);
	  }
	}
      } else if (is_angle_first == "false") {
	for (int angle=0; angle < num_angle; angle++) {
	  for (int height=0; height < num_height; height++) {
	    TactileSensor sensor;
	    sensor.linkName = linkName;
	    sensor.position = origin + cos(init_angle + angle_flag * distance_angle * angle) * direction_x + sin(init_angle + angle_flag * distance_angle * angle) * direction_y + direction_cylinder * height_flag * height * distance_height;
	    sensor.rot = cnoid::VectorXd::Zero(9);
	    cnoid::Matrix3 rotation = rot /*親リンクから曲面の中心（z軸中心・x軸開始位置）までの回転*/* cnoid::rotFromRpy(0,0,init_angle + angle_flag * distance_angle * angle) /*回転中心から各回転位置への変換*/* cnoid::rotFromRpy(0,cnoid::PI / 2, 0.0) /*センサ座標への変換*/;
	    for (int a=0; a<3; a++) {
	      for (int b=0; b<3; b++) {
		sensor.rot[a*3+b] = rotation(a,b);
	      }
	    }
	    sensorConfig.push_back(sensor);
	  }
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
