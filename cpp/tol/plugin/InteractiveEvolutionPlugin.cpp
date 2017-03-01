//
// Created by elte on 20-9-15.
//

#include "InteractiveEvolutionPlugin.h"

#include <gazebo/gui/KeyEventHandler.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/rendering/Visual.hh>

namespace gz = gazebo;

namespace tol
{

InteractiveEvolutionPlugin::InteractiveEvolutionPlugin() :
        GUIPlugin()
{
  // Make the widget invisible - only listening to key events.
  node_.reset(new gz::transport::Node());
  node_->Init();

  std::cout << "Interaction plugin loaded. Press `i` to disable interactive mode,"
          " I (Shift + i) to enable it, and `o` to create offspring "
          "from selected robots. Requesting offspring will enable "
          "interactive mode." << std::endl;

  keyPub_ = node_->Advertise<gz::msgs::Request>("~/request");

  // Register key event handler
  gz::gui::KeyEventHandler::Instance()->AddPressFilter("building_maker",
                                                       boost::bind(&InteractiveEvolutionPlugin::OnKeyDown, this, _1));

  // Set the frame background and foreground colors
  this->setStyleSheet(
          "QFrame { background-color : rgba(100, 100, 100, 255); color : white; }");

  // Create the main layout
  QHBoxLayout *mainLayout = new QHBoxLayout;

  // Create the frame to hold all the widgets
  QFrame *mainFrame = new QFrame();

  // Create the layout that sits inside the frame
  QVBoxLayout *frameLayout = new QVBoxLayout();

  // Create a push button, and connect it to the OnButton function
  QPushButton *button = new QPushButton(tr("Reproduce"));
  connect(button,
          SIGNAL(clicked()),
          this,
          SLOT(OnReproduceButton()));

  // Add the button to the frame's layout
  frameLayout->addWidget(button);

  // Add frameLayout to the frame
  mainFrame->setLayout(frameLayout);

  // Add the frame to the main layout
  mainLayout->addWidget(mainFrame);

  // Remove margins to reduce space
  frameLayout->setContentsMargins(0,
                                  0,
                                  0,
                                  0);
  mainLayout->setContentsMargins(0,
                                 0,
                                 0,
                                 0);

  this->setLayout(mainLayout);

  // Position and resize this widget
  this->move(10,
             10);
  this->resize(120,
               30);
}

InteractiveEvolutionPlugin::~InteractiveEvolutionPlugin()
{
}

void
InteractiveEvolutionPlugin::OnReproduceButton()
{
  gz::gui::MainWindow *mainWindow = gz::gui::get_main_window();
  gz::gui::GLWidget *glWidget = mainWindow->findChild<gz::gui::GLWidget *>("GLWidget");

  auto visuals = glWidget->SelectedVisuals();
  if (visuals.size() != 2) {
    std::cerr << "Wrong number of visuals selected." << std::endl;
    return;
  }

  auto modelVis1 = visuals[0]->GetRootVisual();
  auto modelVis2 = visuals[1]->GetRootVisual();

  if (!modelVis1 || !modelVis2 || modelVis1 == modelVis2) {
    std::cerr << "Incorrect selection (missing model or models identical)" << std::endl;
    return;
  }

  std::cout << "Sending offspring request..." << std::endl;
  gz::msgs::Request req;
  req.set_id(ignition::math::Rand::IntUniform(1,
                                              100000));
  req.set_request("produce_offspring");
  req.set_data(modelVis1->GetName() + "+++" + modelVis2->GetName());
  keyPub_->Publish(req);
}

bool
InteractiveEvolutionPlugin::OnKeyDown(const ::gazebo::common::KeyEvent _event)
{
  if (_event.text == "i" || _event.text == "I") {
    gz::msgs::Request req;
    req.set_id(ignition::math::Rand::IntUniform(1,
                                                100000));
    if (_event.shift) {
      std::cout << "Enabling interaction..." << std::endl;
      req.set_request("enable_interaction");
    } else {
      std::cout << "Disabling interaction..." << std::endl;
      req.set_request("disable_interaction");
    }

    keyPub_->Publish(req);
    return true;
  } else if (_event.text == "o" || _event.text == "O") {
    OnReproduceButton();
    return true;
  }

  return false;
}
}

GZ_REGISTER_GUI_PLUGIN(tol::InteractiveEvolutionPlugin)