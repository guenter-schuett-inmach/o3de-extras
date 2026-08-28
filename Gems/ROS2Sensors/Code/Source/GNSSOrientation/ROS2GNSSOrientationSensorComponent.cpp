/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "ROS2GNSSOrientationSensorComponent.h"
#include <AzCore/Math/Matrix4x4.h>
#include <Source/RigidBodyComponent.h>
#include <ROS2/ROS2NamesBus.h>
#include <Georeferencing/GeoreferenceBus.h>
#include <ROS2/Clock/ROS2ClockRequestBus.h>

namespace ROS2Sensors
{
    namespace
    {
        const char* GNSSOrientationMsgType = "geometry_msgs::msg::QuaternionStamped";
    }

    void ROS2GNSSOrientationSensorComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<ROS2GNSSOrientationSensorComponent, SensorBaseType>()->Version(1);

            if (auto* editContext = serialize->GetEditContext())
            {
                editContext->Class<ROS2GNSSOrientationSensorComponent>("ROS2 GNSS Orientation Sensor", "GNSS orientation sensor component")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                    ->Attribute(AZ::Edit::Attributes::Icon, "Editor/Icons/Components/ROS2GNSSOrientationSensor.svg")
                    ->Attribute(AZ::Edit::Attributes::ViewportIcon, "Editor/Icons/Components/Viewport/ROS2GNSSOrientationSensor.svg")
                    ->Attribute(AZ::Edit::Attributes::Visibility, AZ::Edit::PropertyVisibility::ShowChildrenOnly);
            }
        }
    }

    ROS2GNSSOrientationSensorComponent::ROS2GNSSOrientationSensorComponent()
    {
        ROS2::TopicConfiguration pc;
        pc.m_type = GNSSOrientationMsgType;
        pc.m_topic = "gnss_orientation";
        m_sensorConfiguration.m_frequency = 10;
        m_sensorConfiguration.m_publishersConfigurations.insert(AZStd::make_pair(GNSSOrientationMsgType, pc));
        m_levelRotation = AZ::Quaternion::CreateIdentity();
    }

    ROS2GNSSOrientationSensorComponent::ROS2GNSSOrientationSensorComponent(const ROS2::SensorConfiguration& sensorConfiguration)
    {
        m_sensorConfiguration = sensorConfiguration;
    }

    void ROS2GNSSOrientationSensorComponent::Activate()
    {
        ROS2SensorComponentBase::Activate();
        auto ros2Node = ROS2::ROS2Interface::Get()->GetNode();
        AZ_Assert(m_sensorConfiguration.m_publishersConfigurations.size() == 1, "Invalid configuration of publishers for GPS Oriented sensor");

        const auto publisherConfig = m_sensorConfiguration.m_publishersConfigurations[GNSSOrientationMsgType];
        AZStd::string fullTopic;
        ROS2::ROS2NamesRequestBus::BroadcastResult(
            fullTopic, &ROS2::ROS2NamesRequestBus::Events::GetNamespacedName, GetNamespace(), publisherConfig.m_topic);
        m_GNSSOrientationPublisher = ros2Node->create_publisher<geometry_msgs::msg::QuaternionStamped>(fullTopic.data(), publisherConfig.GetQoS());

        const auto* ros2Frame = GetEntity()->FindComponent<ROS2::ROS2FrameComponent>();
        AZ_Assert(ros2Frame, "Invalid component pointer value");
        m_GNSSOrientationMsg.header.frame_id = ros2Frame->GetNamespacedFrameID().data();

        // get rotation of Level vs. ENU GNSS-Orientation
        Georeferencing::GeoreferenceRequestsBus::BroadcastResult(m_levelRotation, &Georeferencing::GeoreferenceRequests::GetRotationFromLevelToENU);

        StartSensor(
            m_sensorConfiguration.m_frequency,
            [this]([[maybe_unused]] auto&&... args)
            {
                if (!m_sensorConfiguration.m_publishingEnabled)
                {
                    return;
                }
                FrequencyTick();
            });
    }

    void ROS2GNSSOrientationSensorComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("ROS2GNSSSensor"));
    }

    void ROS2GNSSOrientationSensorComponent::Deactivate()
    {
        StopSensor();
        m_GNSSOrientationPublisher.reset();
        ROS2SensorComponentBase::Deactivate();
    }

    void ROS2GNSSOrientationSensorComponent::FrequencyTick()
    {
        AzPhysics::RigidBody* rigidBody = nullptr;
        AZ::EntityId entityId = GetEntityId();
        // fetch current orientation relative to level
        Physics::RigidBodyRequestBus::EventResult(rigidBody, entityId, &Physics::RigidBodyRequests::GetRigidBody);
        if (!rigidBody)
        {
            AZ_Error(
                "ROS2GNSSOrientationSensorComponent",
                false,
                "Entity %s does not have a rigid body - stopping GNSS sensor.",
                entityId.ToString().c_str());
            StopSensor();
        }
        else
        {
            const AZ::Quaternion orientationLevel = rigidBody->GetOrientation();
            // determine current orientation relative to GNSS ENU coordinates
            const AZ::Quaternion orientationENU = m_levelRotation * orientationLevel;

            m_GNSSOrientationMsg.quaternion.x = orientationENU.GetX();
            m_GNSSOrientationMsg.quaternion.y = orientationENU.GetY();
            m_GNSSOrientationMsg.quaternion.z = orientationENU.GetZ();
            m_GNSSOrientationMsg.quaternion.w = orientationENU.GetW();

            m_GNSSOrientationMsg.header.stamp = ROS2::ROS2ClockInterface::Get()->GetROSTimestamp();
            m_GNSSOrientationPublisher->publish(m_GNSSOrientationMsg);
        }
    }

} // namespace ROS2Sensors
