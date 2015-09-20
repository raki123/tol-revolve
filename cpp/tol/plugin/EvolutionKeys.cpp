//
// Created by elte on 20-9-15.
//

#include "EvolutionKeys.h"

#include <gazebo/gui/MouseEventHandler.hh>
#include <gazebo/gui/KeyEventHandler.hh>
#include <gazebo/rendering/Visual.hh>
#include <gazebo/msgs/msgs.hh>

namespace gz = gazebo;

namespace tol {

EvolutionKeys::EvolutionKeys():
GUIPlugin()
{
	// Make the widget invisible - only listening to key events.
	std::cout << "Evolution key plugin loaded. Ctrl + Shift + Left click to select"
						 " parents, then click the button to produce offspring." << std::endl;
	node_.reset(new gz::transport::Node());
	node_->Init();

	keyPub_ = node_->Advertise<gz::msgs::Request>("~/request");

	gz::gui::MouseEventHandler::Instance()->AddPressFilter(
			"evokeys", boost::bind(&EvolutionKeys::OnMousePress, this, _1));

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
	connect(button, SIGNAL(clicked()), this, SLOT(OnButton()));

	// Add the button to the frame's layout
	frameLayout->addWidget(button);

	// Add frameLayout to the frame
	mainFrame->setLayout(frameLayout);

	// Add the frame to the main layout
	mainLayout->addWidget(mainFrame);

	// Remove margins to reduce space
	frameLayout->setContentsMargins(0, 0, 0, 0);
	mainLayout->setContentsMargins(0, 0, 0, 0);

	this->setLayout(mainLayout);

	// Position and resize this widget
	this->move(10, 10);
	this->resize(120, 30);
}

EvolutionKeys::~EvolutionKeys() {
	if (userCam_) {
		userCam_.reset();
	}

	gz::gui::MouseEventHandler::Instance()->RemoveReleaseFilter("evokeys");
}

bool EvolutionKeys::OnMousePress(const gz::common::MouseEvent &_event) {
	if (!userCam_) {
		userCam_ = gz::gui::get_active_camera();
	}

	if (userCam_ && _event.Control() &&
		_event.Shift() &&
		_event.Button() == gz::common::MouseEvent::MouseButton::LEFT &&
		!_event.Dragging()) {

		gz::rendering::VisualPtr vis = userCam_->GetVisual(_event.Pos());
		if (!vis) {
			return false;
		}

		gz::rendering::VisualPtr modelVis = vis->GetRootVisual();
		if (!modelVis) {
			return false;
		}

		gz::msgs::Request req;
		req.set_id(ignition::math::Rand::IntUniform(1, 100000));
		req.set_request("set_evolution_parent");
		req.set_data(modelVis->GetName());
		keyPub_->Publish(req);

		std::cout << "Selected reprod " << modelVis->GetName() << std::endl;
		return true;
	}

	return false;
}

void EvolutionKeys::OnButton() {
	std::cout << "Produce offspring!" << std::endl;
	gz::msgs::Request req;
	req.set_id(ignition::math::Rand::IntUniform(1, 100000));
	req.set_request("produce_offspring");
	keyPub_->Publish(req);
}

}

GZ_REGISTER_GUI_PLUGIN(tol::EvolutionKeys)