#ifndef NEXT_STEP_PANEL_H
#define NEXT_STEP_PANEL_H

#ifndef Q_MOC_RUN
# include <ros/ros.h>
# include <rviz/panel.h>
#endif

class QPushButton;

namespace rviz_plugins
{

class NextStepPanel: public rviz::Panel
{
Q_OBJECT
public:
  NextStepPanel( QWidget* parent = 0 );

  virtual void load( const rviz::Config& config ) override;
  virtual void save( rviz::Config config ) const override;

protected Q_SLOTS:
  void triggerNextStep();

protected:
  QPushButton* next_step_button_;
  ros::NodeHandle nh_;
  ros::Publisher next_step_pub_;
};

} // end namespace rviz_plugins

#endif // NEXT_STEP_PANEL_H
