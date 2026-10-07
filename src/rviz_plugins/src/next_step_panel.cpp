#include <QHBoxLayout>
#include <QPushButton>
#include <std_msgs/Empty.h>
#include "next_step_panel.h"

namespace rviz_plugins
{

NextStepPanel::NextStepPanel( QWidget* parent )
  : rviz::Panel( parent )
{
  QHBoxLayout* layout = new QHBoxLayout;
  next_step_button_ = new QPushButton( "Next Step" );
  layout->addWidget( next_step_button_ );
  setLayout( layout );

  next_step_pub_ = nh_.advertise<std_msgs::Empty>( "/sof_brrt/step", 1 );

  connect( next_step_button_, SIGNAL( clicked() ), this, SLOT( triggerNextStep() ));
}

void NextStepPanel::triggerNextStep()
{
  std_msgs::Empty msg;
  next_step_pub_.publish( msg );
}

void NextStepPanel::load( const rviz::Config& config )
{
  rviz::Panel::load( config );
}

void NextStepPanel::save( rviz::Config config ) const
{
  rviz::Panel::save( config );
}

} // end namespace rviz_plugins

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_plugins::NextStepPanel, rviz::Panel)
