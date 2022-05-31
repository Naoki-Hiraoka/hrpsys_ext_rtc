// -*- C++ -*-
/*!
 * @file  CollisionDetector.h
 * @brief collision detector component
 * @date  $Date$
 *
 * $Id$
 */

#ifndef COLLISION_DETECTOR2_H
#define COLLISION_DETECTOR2_H

#include <rtm/idl/BasicDataType.hh>
#include "hrpsys/idl/HRPDataTypes.hh"
#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>
#include <rtm/idl/BasicDataTypeSkel.h>
#include <cnoid/Body>
#include <choreonoid_vclip/choreonoid_vclip.h>
#include <unordered_map>
#include <cnoid/BodyCollisionDetector>
#include <cnoid/AISTCollisionDetector>

#include "CollisionDetector2Service_impl.h"
#include "../SoftErrorLimiter2/beep.h"

using namespace RTC;

class CollisionDetector2
  : public RTC::DataFlowComponentBase
{
 public:
  /**
     \brief Constructor
     \param manager pointer to the Manager
  */
  CollisionDetector2(RTC::Manager* manager);
  /**
     \brief Destructor
  */
  virtual ~CollisionDetector2();

  // The initialize action (on CREATED->ALIVE transition)
  // formaer rtc_init_entry()
  virtual RTC::ReturnCode_t onInitialize();

  // The activated action (Active state entry action)
  // former rtc_active_entry()
  virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);

  // The deactivated action (Active state exit action)
  // former rtc_active_exit()
  virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);

  // The execution action that is invoked periodically
  // former rtc_active_do()
  virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

  bool setTolerance(const char *i_link_pair_name, double i_tolerance);
  bool setCollisionLoop(int input_loop);
  bool getCollisionStatus(OpenHRP::CollisionDetectorService::CollisionState &state);

  bool checkIsSafeTransition(void);
  bool enable(void);
  bool disable(void);

 protected:
  // Configuration variable declaration
  // <rtc-template block="config_declare">
  
  // </rtc-template>

  // DataInPort declaration
  // <rtc-template block="inport_declare">
  TimedDoubleSeq m_qRef;
  InPort<TimedDoubleSeq> m_qRefIn;
  OpenHRP::TimedLongSeqSeq m_servoState;
  InPort<OpenHRP::TimedLongSeqSeq> m_servoStateIn;
  
  // </rtc-template>

  // DataOutPort declaration
  // <rtc-template block="outport_declare">
  TimedDoubleSeq m_q;
  OutPort<TimedDoubleSeq> m_qOut;
  TimedLongSeq m_beepCommand;
  OutPort<TimedLongSeq> m_beepCommandOut;
  
  // </rtc-template>

  // CORBA Port declaration
  // <rtc-template block="corbaport_declare">
  RTC::CorbaPort m_CollisionDetector2ServicePort;
  
  // </rtc-template>

  // Service declaration
  // <rtc-template block="service_declare">
  CollisionDetector2Service_impl m_service0;
  
  // </rtc-template>

  // Consumer declaration
  // <rtc-template block="consumer_declare">

  
  // </rtc-template>
  void setupVClipModel(cnoid::BodyPtr i_body);
  void setupVClipModel(cnoid::Link *i_link);

 private:
  class CollisionLinkPair {
  public:
    CollisionLinkPair(cnoid::LinkPtr link0_, cnoid::LinkPtr link1_) : link0(link0_), link1(link1_), point0(cnoid::Vector3(0,0,0)), point1(cnoid::Vector3(0,0,0)), distance(0), tolerance(0) {
    }
    cnoid::LinkPtr link0, link1;
    cnoid::Vector3 point0, point1; // world coords
    double distance;
    double tolerance;
  };
  std::unordered_map<cnoid::LinkPtr, std::shared_ptr<Vclip::Polyhedron> > m_VclipLinks;
  std::vector<int> m_curr_collision_mask, m_init_collision_mask;
  cnoid::BodyPtr m_robot;
  std::map<std::string, std::shared_ptr<CollisionLinkPair> > m_pair;
  int m_loop_for_check, m_collision_loop;

  std::vector<bool> m_link_collision;
  unsigned int m_debugLevel;
  bool m_enable;

  double collision_beep_freq; // [Hz]
  int  collision_beep_count;
  OpenHRP::CollisionDetectorService::CollisionState m_state;
  BeepClient bc;
  // Since this RTC is stable RTC, we support both direct beeping from this RTC and beepring through BeeperRTC.
  // If m_beepCommand is connected to BeeperRTC, is_beep_port_connected is true.
  bool is_beep_port_connected;

  bool collision_mode;
  cnoid::VectorXd m_stop_jointdata;
  bool m_safe_posture;

  cnoid::BodyCollisionDetector m_bodyCollisionDetector;
};

extern "C"
{
  void CollisionDetector2Init(RTC::Manager* manager);
};

#endif // COLLISION_DETECTOR2_H
