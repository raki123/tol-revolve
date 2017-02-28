//
// Created by elte on 20-9-15.
//

#ifndef TRIANGLEOFLIFE_EVOLUTIONKEYS_H
#define TRIANGLEOFLIFE_EVOLUTIONKEYS_H

#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829

# include <gazebo/rendering/UserCamera.hh>
# include <gazebo/common/Plugin.hh>
# include <gazebo/gui/GuiPlugin.hh>
# include <gazebo/transport/transport.hh>
# include <gazebo/gui/gui.hh>
# include <gazebo/common/KeyEvent.hh>

#endif

namespace tol
{

class InteractiveEvolutionPlugin
        : public ::gazebo::GUIPlugin
{
Q_OBJECT

public:

    InteractiveEvolutionPlugin();

    ~InteractiveEvolutionPlugin();

protected slots:

    void
    OnReproduceButton();

private:
    bool
    OnKeyDown(const ::gazebo::common::KeyEvent _event);

    // Transport nodes for the contact messages
    ::gazebo::transport::NodePtr node_;

    // Key publisher
    ::gazebo::transport::PublisherPtr keyPub_;
};

}


#endif //TRIANGLEOFLIFE_EVOLUTIONKEYS_H
