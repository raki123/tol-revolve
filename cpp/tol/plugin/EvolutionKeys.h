//
// Created by elte on 20-9-15.
//

#ifndef TRIANGLEOFLIFE_EVOLUTIONKEYS_H
#define TRIANGLEOFLIFE_EVOLUTIONKEYS_H

#include <gazebo/rendering/UserCamera.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gui/GuiPlugin.hh>
#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
# include <gazebo/transport/transport.hh>
# include <gazebo/gui/gui.hh>
#endif

namespace tol {

class EvolutionKeys : public ::gazebo::GUIPlugin {
	Q_OBJECT
public:

	EvolutionKeys();
	~EvolutionKeys();

protected slots:
	void OnButton();

private:
	// Holds instance of the active camera
	::gazebo::rendering::UserCameraPtr userCam_;

	// Transport nodes for the contact messages
	::gazebo::transport::NodePtr node_;

	// Key publisher
	::gazebo::transport::PublisherPtr keyPub_;

	/// \brief Callback for a mouse release event.
	/// \param[in] _event The mouse release event
	/// \return True if handled by this function.
	bool OnMousePress(const ::gazebo::common::MouseEvent &_event);
};

}


#endif //TRIANGLEOFLIFE_EVOLUTIONKEYS_H
