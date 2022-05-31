// -*- C++ -*-
/*!
 * @file  CollisionDetector.cpp
 * @brief collisoin detector component
 * $Date$
 *
 * $Id$
 */

#include <iomanip>
#include <rtm/CorbaNaming.h>
#include <cnoid/BodyLoader>
#include <hrpsys/idl/RobotHardwareService.hh>
#include "CollisionDetector2.h"
#include <choreonoid_qhull/choreonoid_qhull.h>

#define deg2rad(x)	((x)*M_PI/180)
#define rad2deg(x)      ((x)*180/M_PI)

// Module specification
// <rtc-template block="module_spec">
static const char* component_spec[] =
{
    "implementation_id", "CollisionDetector2",
    "type_name",         "CollisionDetector2",
    "description",       "collisoin detector component",
    "version",           "0.0.0",
    "vendor",            "Naoki-Hiraoka",
    "category",          "example",
    "activity_type",     "DataFlowComponent",
    "max_instance",      "10",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    "conf.default.debugLevel", "0",
    ""
};
// </rtc-template>

static std::ostream& operator<<(std::ostream& os, const struct RTC::Time &tm)
{
    int pre = os.precision();
    os.setf(std::ios::fixed);
    os << std::setprecision(6)
       << (tm.sec + tm.nsec/1e9)
       << std::setprecision(pre);
    os.unsetf(std::ios::fixed);
    return os;
}

CollisionDetector2::CollisionDetector2(RTC::Manager* manager)
    : RTC::DataFlowComponentBase(manager),
      m_qRefIn("qRef", m_qRef),
      m_servoStateIn("servoStateIn", m_servoState),
      m_qOut("q", m_q),
      m_beepCommandOut("beepCommand", m_beepCommand),
      m_CollisionDetector2ServicePort("CollisionDetector2Service"),
      m_loop_for_check(0),
      m_collision_loop(1),
      m_debugLevel(0),
      m_enable(true),
      is_beep_port_connected(false),
      collision_mode(false)
{
    m_service0.collision(this);
    init_beep();
    start_beep(3136);
}

CollisionDetector2::~CollisionDetector2()
{
  quit_beep();
}



RTC::ReturnCode_t CollisionDetector2::onInitialize()
{
    std::cerr << "[" << m_profile.instance_name << "] onInitialize()" << std::endl;
    // <rtc-template block="bind_config">
    // Bind variables and configuration variable
    bindParameter("debugLevel", m_debugLevel, "0");

    // </rtc-template>

    // Registration: InPort/OutPort/Service
    // <rtc-template block="registration">
    // Set InPort buffers
    addInPort("qRef", m_qRefIn);
    addInPort("servoStateIn", m_servoStateIn);

    // Set OutPort buffer
    addOutPort("q", m_qOut);
    addOutPort("beepCommand", m_beepCommandOut);

    // Set service provider to Ports
    m_CollisionDetector2ServicePort.registerProvider("service0", "CollisionDetector2Service", m_service0);

    // Set service consumers to Ports

    // Set CORBA Service Ports
    addPort(m_CollisionDetector2ServicePort);

    // </rtc-template>

    // load robot
    cnoid::BodyLoader bodyLoader;
    std::string fileName;
    if(this->getProperties().hasKey("model")) fileName = std::string(this->getProperties()["model"]);
    else fileName = std::string(this->m_pManager->getConfig()["model"]); // 引数 -o で与えたプロパティを捕捉
    std::cerr << "[" << this->m_profile.instance_name << "] model: " << fileName <<std::endl;
    this->m_robot = bodyLoader.load(fileName);
    if(!this->m_robot){
      std::cerr << "\x1b[31m[" << m_profile.instance_name << "] " << "failed to load model[" << fileName << "]" << "\x1b[39m" << std::endl;
      return RTC::RTC_ERROR;
    }

    std::string collisionModelStr;
    if(this->getProperties().hasKey("collision_model")) collisionModelStr = std::string(this->getProperties()["collision_model"]);
    else collisionModelStr = std::string(this->m_pManager->getConfig()["collision_model"]); // 引数 -o で与えたプロパティを捕捉
    std::cerr << "[" << this->m_profile.instance_name << "] collision_model: " << collisionModelStr <<std::endl;
    if ( collisionModelStr == "convex hull" ||
         collisionModelStr == "" ) { // set convex hull as default
      //choreonoid_qhull::convertAllCollisionToConvexHull(this->m_robot);
    }
    setupVClipModel(this->m_robot);

    std::string collisionPairStr;
    if ( this->getProperties().hasKey("collision_pair")) collisionPairStr = std::string(this->getProperties()["collision_pair"]);
    else if(this->m_pManager->getConfig().hasKey("collision_pair")) collisionPairStr = std::string(this->m_pManager->getConfig()["collision_pair"]); // 引数 -o で与えたプロパティを捕捉
    std::cerr << "[" << m_profile.instance_name << "] prop[collision_pair] ->" << collisionPairStr << std::endl;
    std::istringstream iss(collisionPairStr);
    std::string tmp;
    while (getline(iss, tmp, ' ')) {
      size_t pos = tmp.find_first_of(':');
      std::string name1 = tmp.substr(0, pos), name2 = tmp.substr(pos+1);
      if ( m_robot->link(name1)==NULL ) {
        std::cerr << "[" << m_profile.instance_name << "] Could not find robot link " << name1 << std::endl;
        std::cerr << "[" << m_profile.instance_name << "] please choose one of following :";
        for (unsigned int i=0; i < m_robot->numLinks(); i++) {
          std::cerr << " " << m_robot->link(i)->name();
        }
        std::cerr << std::endl;
        continue;
      }
      if ( m_robot->link(name2)==NULL ) {
        std::cerr << "[" << m_profile.instance_name << "] Could not find robot link " << name2 << std::endl;
        std::cerr << "[" << m_profile.instance_name << "] please choose one of following :";
        for (unsigned int i=0; i < m_robot->numLinks(); i++) {
          std::cerr << " " << m_robot->link(i)->name();
        }
        std::cerr << std::endl;
        continue;
      }
      std::cerr << "[" << m_profile.instance_name << "] check collisions between " << m_robot->link(name1)->name() << " and " <<  m_robot->link(name2)->name() << std::endl;
      m_pair[tmp] = std::make_shared<CollisionLinkPair>(m_robot->link(name1), m_robot->link(name2));
    }

    std::string collisionLoopStr = "1";
    if ( this->getProperties().hasKey("collision_pair")) collisionPairStr = std::string(this->getProperties()["collision_pair"]);
    else if( this->m_pManager->getConfig().hasKey("collision_pair") ) collisionPairStr = std::string(this->m_pManager->getConfig()["collision_pair"]); // 引数 -o で与えたプロパティを捕捉
    this->m_collision_loop = std::stoi(collisionLoopStr);
    std::cerr << "[" << m_profile.instance_name << "] set collision_loop: " << m_collision_loop << std::endl;

    // true (1) do not move when collide,
    // false(0) move even if collide
    this->m_curr_collision_mask.resize(m_robot->numJoints(), 0); // collision_mask used to select output                0: passthough reference data, 1 output safe data
    this->m_init_collision_mask.resize(m_robot->numJoints(), 1); // collision_mask defined in .conf as [collisoin_mask] 0: move even if collide, 1: do not move when collide
    std::string collisionMaskStr;
    if ( this->getProperties().hasKey("collision_mask")) collisionMaskStr = std::string(this->getProperties()["collision_mask"]);
    else if( this->m_pManager->getConfig().hasKey("collision_mask") ) collisionMaskStr = std::string(this->m_pManager->getConfig()["collision_mask"]); // 引数 -o で与えたプロパティを捕捉
    if ( collisionMaskStr != "" ) {
      std::cerr << "[" << m_profile.instance_name << "] prop[collision_mask] ->" << collisionMaskStr << std::endl;
      coil::vstring mask_str = coil::split(collisionMaskStr, ",");
      if (mask_str.size() == m_robot->numJoints()) {
        for (size_t i = 0; i < m_robot->numJoints(); i++) {coil::stringTo(this->m_init_collision_mask[i], mask_str[i].c_str()); }
        for (size_t i = 0; i < m_robot->numJoints(); i++) {
          if ( this->m_init_collision_mask[i] == 0 ) {
            std::cerr << "[" << m_profile.instance_name << "] CollisionDetector2 will not control " << this->m_robot->joint(i)->name() << std::endl;
          }
        }
      }else{
        std::cerr << "[" << m_profile.instance_name << "] ERROR size of collision_mask is differ from robot joint number .. " << mask_str.size()  << ", " << m_robot->numJoints() << std::endl;
      }
    }

    // setup collision state
    this->m_state.angle.length(m_robot->numJoints());
    this->m_state.collide.length(m_robot->numLinks());

    // allocate memory for outPorts
    this->m_q.data.length(this->m_robot->numJoints());
    for(unsigned int i=0; i<this->m_robot->numJoints(); i++){
      this->m_q.data[i] = 0;
    }
    this->m_servoState.data.length(this->m_robot->numJoints());
    for(unsigned int i = 0; i < this->m_robot->numJoints(); i++) {
      this->m_servoState.data[i].length(1);
      int status = 0;
      status |= 1<< OpenHRP::RobotHardwareService::CALIB_STATE_SHIFT;
      status |= 1<< OpenHRP::RobotHardwareService::POWER_STATE_SHIFT;
      status |= 1<< OpenHRP::RobotHardwareService::SERVO_STATE_SHIFT;
      status |= 0<< OpenHRP::RobotHardwareService::SERVO_ALARM_SHIFT;
      status |= 0<< OpenHRP::RobotHardwareService::DRIVER_TEMP_SHIFT;
      this->m_servoState.data[i][0] = status;
    }

    this->m_safe_posture = true;
    this->m_stop_jointdata = cnoid::VectorXd::Zero(m_robot->numJoints());
    this->m_link_collision.resize(m_robot->numLinks(),false);

    this->collision_beep_freq = 3.0; // 3 times / 1[s]
    this->collision_beep_count = 0;
    m_beepCommand.data.length(bc.get_num_beep_info());

    this->m_bodyCollisionDetector.setCollisionDetector(new cnoid::AISTCollisionDetector);
    this->m_bodyCollisionDetector.addBody(this->m_robot, true);
    this->m_bodyCollisionDetector.makeReady();

    return RTC::RTC_OK;
}

RTC::ReturnCode_t CollisionDetector2::onActivated(RTC::UniqueId ec_id)
{
    std::cerr << "[" << m_profile.instance_name<< "] onActivated(" << ec_id << ")" << std::endl;
    m_safe_posture = false;
    m_loop_for_check = 0;
    collision_mode = false;

    return RTC::RTC_OK;
}

RTC::ReturnCode_t CollisionDetector2::onDeactivated(RTC::UniqueId ec_id)
{
    std::cerr << "[" << m_profile.instance_name<< "] onDeactivated(" << ec_id << ")" << std::endl;
    return RTC::RTC_OK;
}

RTC::ReturnCode_t CollisionDetector2::onExecute(RTC::UniqueId ec_id)
{
    static int loop = 0;
    loop++;

    double dt = 1.0 / this->get_context(ec_id)->get_rate();

    // Connection check of m_beepCommand to BeeperRTC
    //   If m_beepCommand is not connected to BeeperRTC and sometimes, check connection.
    //   If once connection is found, never check connection.
    if (!is_beep_port_connected && (loop % 500 == 0) ) {
      if ( m_beepCommandOut.connectors().size() > 0 ) {
        is_beep_port_connected = true;
        quit_beep();
        std::cerr << "[" << m_profile.instance_name<< "] beepCommand data port connection found! Use BeeperRTC." << std::endl;
      }
    }

    /*
      サーボオン状態の関節があるときに、次の指令値が自己干渉しない姿勢->自己干渉する姿勢に遷移するものだった場合、collision modeが作動する.
      collision mode時は、collision_maskが1の関節は動かない. (stop_jointdata)
      全軸がサーボオフだと、collision modeが解消される
     */

    if (m_qRefIn.isNew() ) {
      m_qRefIn.read();
    }
    if (m_servoStateIn.isNew()) {
        m_servoStateIn.read();
    }
    if ( ! m_enable && m_qRef.data.length() == m_robot->numJoints()) {
        if ( loop % 100 == 1) {
            std::cerr << "[" << m_profile.instance_name << "] CAUTION!! The robot is moving without checking self collision detection!!! please send enableCollisionDetection to CollisoinDetection RTC" << std::endl;
        }
        for ( unsigned int i = 0; i < m_q.data.length(); i++ ) {
          m_q.data[i] = m_qRef.data[i];
        }
        m_q.tm = m_qRef.tm;
        m_qOut.write();
    }

    if ( m_enable && m_qRef.data.length() == m_robot->numJoints()) {

      // check servo
      bool has_servoOn = false;
      for (unsigned int i = 0; i < m_robot->numJoints(); i++ ){
        int servo_state = (m_servoState.data[i][0] & OpenHRP::RobotHardwareService::SERVO_STATE_MASK) >> OpenHRP::RobotHardwareService::SERVO_STATE_SHIFT;
        has_servoOn = has_servoOn || (servo_state == 1);
      }

      //set robot model's angle for collision check(two types)
      //  1. ! collision_mode  .. check based on qRef
      //  2. collision_mode    .. check based on stop_jointdata
      if ( m_loop_for_check == 0 ) { // update robot posutre for each m_loop_for_check timing
        if (!this->collision_mode ) {
          for ( unsigned int i = 0; i < m_robot->numJoints(); i++ ){
            m_robot->joint(i)->q() = m_qRef.data[i];
          }
        }else{
          for ( unsigned int i = 0; i < m_robot->numJoints(); i++ ){
            if ( m_curr_collision_mask[i] == 1) {// joint with 1 (do not move when collide :default), need to be updated using stop data
              m_robot->joint(i)->q() = m_stop_jointdata[i];
            }else{                               // joint with 0 (move even if collide), need to be updated using reference(dangerous) data
              m_robot->joint(i)->q() = m_qRef.data[i];
            }
          }
        }
        m_robot->calcForwardKinematics();
      }

      coil::TimeValue tm1 = coil::gettimeofday();

      // m_collision_loop 回の周期に分けて、干渉をチェックする.
      // {
      //   std::map<std::string, std::shared_ptr<CollisionLinkPair> >::iterator it = this->m_pair.begin();
      //   for ( int i = 0; it != m_pair.end(); it++, i++){
      //     int sub_size = (m_pair.size() + m_collision_loop -1) / m_collision_loop;  // 10 / 3 = 3  / floor
      //     // 0 : 0 .. sub_size-1                            // 0 .. 2
      //     // 1 : sub_size ... sub_size*2-1                  // 3 .. 5
      //     // k : sub_size*k ... sub_size*(k+1)-1            // 6 .. 8
      //     // n : sub_size*n ... m_pair.size()               // 9 .. 10
      //     if ( sub_size*m_loop_for_check <= i && i < sub_size*(m_loop_for_check+1) ) {
      //       std::shared_ptr<CollisionLinkPair> c = it->second;
      //       cnoid::Vector3 point0_local, point1_local;
      //       choreonoid_vclip::computeDistance(this->m_VclipLinks[c->link0], c->link0->p(), c->link0->R(),
      //                                         this->m_VclipLinks[c->link1], c->link1->p(), c->link1->R(),
      //                                         c->distance, point0_local, point1_local);
      //       c->point0 = c->link0->T() * point0_local;
      //       c->point1 = c->link1->T() * point1_local;
      //     }
      //   }
      // }

      for(std::map<std::string, std::shared_ptr<CollisionLinkPair> >::iterator it = m_pair.begin(); it!=m_pair.end();it++){
        it->second->point0 = it->second->link0->p();
        it->second->point1 = it->second->link1->p();
        it->second->distance = 1.0;
      }
      this->m_bodyCollisionDetector.updatePositions();
      this->m_bodyCollisionDetector.detectCollisions([&](const cnoid::CollisionPair& collisionPair){
          cnoid::Link* link0 = static_cast<cnoid::Link*>(collisionPair.object(0));
          cnoid::Link* link1 = static_cast<cnoid::Link*>(collisionPair.object(1));
          std::string name = link0->name() + ":" + link1->name();
          if(m_pair.find(name) == m_pair.end()) return;
          std::shared_ptr<CollisionLinkPair> c = m_pair[name];
          if(collisionPair.collisions().size() == 0) return;
          c->point0 = c->point1 = collisionPair.collisions()[0].point;
          c->distance = -collisionPair.collisions()[0].depth;
        });

      coil::TimeValue tm2 = coil::gettimeofday();

      if( !has_servoOn ) {
        this->collision_mode = false;
      }

      // 干渉チェックが1周したら、チェック結果に応じてstateを変更する
      std::vector<std::pair<cnoid::Vector3, cnoid::Vector3> > lines;
      if ( m_loop_for_check == m_collision_loop-1 ) {
        bool last_safe_posture = this->m_safe_posture;
        this->m_safe_posture = true;
        for (unsigned int i = 0; i < m_robot->numLinks(); i++ ){
          this->m_link_collision[m_robot->link(i)->index()] = false;
        }
        {
          std::map<std::string, std::shared_ptr<CollisionLinkPair> >::iterator it = m_pair.begin();
          for (unsigned int i = 0; it != m_pair.end(); i++, it++){
            std::shared_ptr<CollisionLinkPair> c = it->second;
            lines.push_back(std::make_pair(c->point0, c->point1));
            if ( c->distance <= c->tolerance ) { // collide
              this->m_safe_posture = false;
              if ( loop%200==0 || last_safe_posture ) {
                std::cerr << "[" << m_profile.instance_name << "] " << i << "/" << m_pair.size() << " pair: " << c->link0->name() << "/" << c->link1->name() <<", distance = " << c->distance << std::endl;
              }
              m_link_collision[c->link0->index()] = true;
              m_link_collision[c->link1->index()] = true;
              std::copy(m_init_collision_mask.begin(), m_init_collision_mask.end(), m_curr_collision_mask.begin()); // copy init_collision_mask to curr_collision_mask
            }
          }
        }

        if(has_servoOn && last_safe_posture && !this->m_safe_posture) {
          this->collision_mode = true;
          for ( unsigned int i = 0; i < m_qRef.data.length(); i++ ) {
            m_stop_jointdata[i] = m_qRef.data[i];;
          }
        }

        // set collisoin state
        m_state.time = tm2;
        for (unsigned int i = 0; i < m_robot->numJoints(); i++ ){
          m_state.angle[i] = m_robot->joint(i)->q();
        }
        for (unsigned int i = 0; i < m_robot->numLinks(); i++ ){
          m_state.collide[i] = m_link_collision[i];
        }
        m_state.lines.length(lines.size());
        for(unsigned int i = 0; i < lines.size(); i++ ){
          const std::pair<cnoid::Vector3, cnoid::Vector3>& line = lines[i];
          double *v;
          m_state.lines[i].length(2);
          m_state.lines[i].get_buffer()[0].length(3);
          v = m_state.lines[i].get_buffer()[0].get_buffer();
          v[0] = line.first.data()[0];
          v[1] = line.first.data()[1];
          v[2] = line.first.data()[2];
          m_state.lines[i].get_buffer()[1].length(3);
          v = m_state.lines[i].get_buffer()[1].get_buffer();
          v[0] = line.second.data()[0];
          v[1] = line.second.data()[1];
          v[2] = line.second.data()[2];
        }
        m_state.computation_time = (tm2-tm1)*1000.0;
        m_state.safe_posture = m_safe_posture;
        m_state.recover_time = 0;
        m_state.loop_for_check = m_loop_for_check;
      }

      // mode に応じて出力
      if (m_safe_posture){ // safe mode
        for ( unsigned int i = 0; i < m_q.data.length(); i++ ) {
          m_q.data[i] = m_qRef.data[i];
        }
      } else {
        for ( unsigned int i = 0; i < m_q.data.length(); i++ ) {
          if (m_curr_collision_mask[i] == 0) { // 0: passthough reference data, 1 output safe data, stop joints only joint with 1
            m_q.data[i] = m_qRef.data[i];
          }else{
            m_q.data[i] = m_stop_jointdata[i];
          }
        }
      }

      if ( m_pair.size() == 0 && (loop % ((int)(5/dt))) == 1 ) {
        std::cerr << "[" << m_profile.instance_name << "] CAUTION!! The robot is moving without checking self collision detection!!! please define collision_pair in configuration file" << std::endl;
      }
      if ( ! has_servoOn && !this->m_safe_posture && !this->collision_mode ) {
        if ( (loop % ((int)(5/dt))) == 1) {
          std::cerr << "[" << m_profile.instance_name << "] CAUTION!! The robot is moving while collision detection!!!, since we do not get safe_posture yet" << std::endl;
        }
      }
      //
      m_q.tm = m_qRef.tm;
      m_qOut.write();

      // beep sound for collision alert
      if ( !m_safe_posture && has_servoOn ) { // If collided and some joint is servoOn
        int beep_loop = 1.0 / dt / collision_beep_freq;
        if (is_beep_port_connected) {
          if ( collision_beep_count % beep_loop == 0 && collision_beep_count % (beep_loop * 3) != 0 ) bc.startBeep(2352, beep_loop*0.7);
          else bc.stopBeep();
        } else {
          if ( collision_beep_count % beep_loop == 0 && collision_beep_count % (beep_loop * 3) != 0 ) start_beep(2352, beep_loop*0.7);
          else stop_beep();
        }
        collision_beep_count++;
      } else {
        if (is_beep_port_connected) bc.stopBeep();
        collision_beep_count = 0;
      }

      if ( ++m_loop_for_check >= m_collision_loop ) m_loop_for_check = 0;

    }
    if (is_beep_port_connected) {
      bc.setDataPort(m_beepCommand);
      if (bc.isWritable()) m_beepCommandOut.write();
    }
    return RTC::RTC_OK;
}

/*
  RTC::ReturnCode_t CollisionDetector2::onAborting(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t CollisionDetector2::onError(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t CollisionDetector2::onReset(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t CollisionDetector2::onStateUpdate(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t CollisionDetector2::onRateChanged(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

bool CollisionDetector2::setTolerance(const char *i_link_pair_name, double i_tolerance) {
    if (strcmp(i_link_pair_name, "all") == 0 || strcmp(i_link_pair_name, "ALL") == 0){
      for ( std::map<std::string, std::shared_ptr<CollisionLinkPair> >::iterator it = m_pair.begin();  it != m_pair.end(); it++){
        it->second->tolerance = i_tolerance;
      }
    }else if ( m_pair.find(std::string(i_link_pair_name)) != m_pair.end() ) {
      m_pair[std::string(i_link_pair_name)]->tolerance = i_tolerance;
    }else{
      return false;
    }
    return true;
}

bool CollisionDetector2::setCollisionLoop(int input_loop) {
    if (input_loop > 0) {
        m_collision_loop = input_loop;
        return true;
    }
    return false;
}

bool CollisionDetector2::getCollisionStatus(OpenHRP::CollisionDetectorService::CollisionState &state)
{
    state = m_state;
    return true;
}

void CollisionDetector2::setupVClipModel(cnoid::BodyPtr i_body)
{
  for (unsigned int i=0; i<i_body->numLinks(); i++) {
    this->m_VclipLinks[i_body->link(i)] = choreonoid_vclip::convertToVClipModel(i_body->link(i)->collisionShape());
  }
}

bool CollisionDetector2::checkIsSafeTransition(void)
{
    for ( unsigned int i = 0; i < m_q.data.length(); i++ ) {
        // If servoOn, check too large joint angle gap. Otherwise (servoOff), neglect too large joint angle gap.
        int servo_state = (m_servoState.data[i][0] & OpenHRP::RobotHardwareService::SERVO_STATE_MASK) >> OpenHRP::RobotHardwareService::SERVO_STATE_SHIFT; // enum SwitchStatus {SWITCH_ON, SWITCH_OFF};
        if (servo_state == 1 && abs(m_q.data[i] - m_qRef.data[i]) > 0.017) return false;
    }
    return true;
}

bool CollisionDetector2::enable(void)
{
    if (m_enable){
        std::cerr << "[" << m_profile.instance_name << "] CollisionDetector2 is already enabled." << std::endl;
        return true;
    }

    if (!checkIsSafeTransition()){
        std::cerr << "[" << m_profile.instance_name << "] CollisionDetector2 cannot be enabled because of different reference joint angle" << std::endl;
        return false;
    }

    std::cerr << "[" << m_profile.instance_name << "] CollisionDetector2 is successfully enabled." << std::endl;

    m_safe_posture = false;
    m_loop_for_check = 0;
    collision_mode = false;

    m_enable = true;
    return true;
}

bool CollisionDetector2::disable(void)
{
    if (!checkIsSafeTransition()){
        std::cerr << "[" << m_profile.instance_name << "] CollisionDetector2 cannot be disabled because of different reference joint angle" << std::endl;
        return false;
    }
    std::cerr << "[" << m_profile.instance_name << "] CollisionDetector2 is successfully disabled." << std::endl;
    m_enable = false;
    return true;
}

extern "C"
{

    void CollisionDetector2Init(RTC::Manager* manager)
    {
        RTC::Properties profile(component_spec);
        manager->registerFactory(profile,
                                 RTC::Create<CollisionDetector2>,
                                 RTC::Delete<CollisionDetector2>);
    }

};


