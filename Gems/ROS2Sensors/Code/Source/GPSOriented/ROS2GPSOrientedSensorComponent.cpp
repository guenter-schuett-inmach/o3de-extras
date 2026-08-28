/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "ROS2GPSOrientedSensorComponent.h"
#include <AzCore/Math/Matrix4x4.h>

#include <Source/RigidBodyComponent.h>

#include <ROS2/ROS2NamesBus.h>

#include <Georeferencing/GeoreferenceBus.h>
#include <ROS2Sensors/GPSOriented/GPSOrientedPostProcessingRequestBus.h>
#include <ROS2/Clock/ROS2ClockRequestBus.h>

namespace ROS2Sensors
{
    namespace
    {
        const char* GPSOrientedMsgType = "gps_msgs::msg::GPSFix";
    }

    void ROS2GPSOrientedSensorComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<ROS2GPSOrientedSensorComponent, SensorBaseType>()->Version(1);

            if (auto* editContext = serialize->GetEditContext())
            {
                editContext->Class<ROS2GPSOrientedSensorComponent>("ROS2 GPS Oriented Sensor", "GPS oriented sensor component")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                    ->Attribute(AZ::Edit::Attributes::Icon, "Editor/Icons/Components/ROS2GPSOrientedSensor.svg")
                    ->Attribute(AZ::Edit::Attributes::ViewportIcon, "Editor/Icons/Components/Viewport/ROS2GPSOrientedSensor.svg")
                    ->Attribute(AZ::Edit::Attributes::Visibility, AZ::Edit::PropertyVisibility::ShowChildrenOnly);
            }
        }
    }

    ROS2GPSOrientedSensorComponent::ROS2GPSOrientedSensorComponent()
    {
        ROS2::TopicConfiguration pc;
        pc.m_type = GPSOrientedMsgType;
        pc.m_topic = "gps_oriented";
        m_sensorConfiguration.m_frequency = 10;
        m_sensorConfiguration.m_publishersConfigurations.insert(AZStd::make_pair(GPSOrientedMsgType, pc));
        m_levelRotation = AZ::Quaternion::CreateIdentity();
    }

    ROS2GPSOrientedSensorComponent::ROS2GPSOrientedSensorComponent(const ROS2::SensorConfiguration& sensorConfiguration)
    {
        m_sensorConfiguration = sensorConfiguration;
    }

    void ROS2GPSOrientedSensorComponent::Activate()
    {
        ROS2SensorComponentBase::Activate();
        auto ros2Node = ROS2::ROS2Interface::Get()->GetNode();
        AZ_Assert(m_sensorConfiguration.m_publishersConfigurations.size() == 1, "Invalid configuration of publishers for GPS Oriented sensor");

        const auto publisherConfig = m_sensorConfiguration.m_publishersConfigurations[GPSOrientedMsgType];
        AZStd::string fullTopic;
        ROS2::ROS2NamesRequestBus::BroadcastResult(
            fullTopic, &ROS2::ROS2NamesRequestBus::Events::GetNamespacedName, GetNamespace(), publisherConfig.m_topic);
        m_gpsOrientedPublisher = ros2Node->create_publisher<gps_msgs::msg::GPSFix>(fullTopic.data(), publisherConfig.GetQoS());

        const auto* ros2Frame = GetEntity()->FindComponent<ROS2::ROS2FrameComponent>();
        AZ_Assert(ros2Frame, "Invalid component pointer value");
        m_gpsOrientedMsg.header.frame_id = ros2Frame->GetNamespacedFrameID().data();

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

    void ROS2GPSOrientedSensorComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("ROS2GNSSSensor"));
    }

    void ROS2GPSOrientedSensorComponent::Deactivate()
    {
        StopSensor();
        m_gpsOrientedPublisher.reset();
        ROS2SensorComponentBase::Deactivate();
    }

    void ROS2GPSOrientedSensorComponent::FrequencyTick()
    {
        using namespace Georeferencing;
        AZ::Vector3 currentPosition{ 0.0f };
        AZ::TransformBus::EventResult(currentPosition, GetEntityId(), &AZ::TransformBus::Events::GetWorldTranslation);

        WGS::WGS84Coordinate currentPositionWGS84;
        GeoreferenceRequestsBus::BroadcastResult(currentPositionWGS84, &GeoreferenceRequests::ConvertFromLevelToWGS84, currentPosition);

        m_gpsOrientedMsg.latitude = currentPositionWGS84.m_latitude;
        m_gpsOrientedMsg.longitude = currentPositionWGS84.m_longitude;
        m_gpsOrientedMsg.altitude = currentPositionWGS84.m_altitude;

        AZ::Vector3 eulerAngles(0.0);
        AzPhysics::RigidBody* rigidBody = nullptr;
        AZ::EntityId entityId = GetEntityId();
        // fetch current orientation relative to level
        Physics::RigidBodyRequestBus::EventResult(rigidBody, entityId, &Physics::RigidBodyRequests::GetRigidBody);
        if (!rigidBody)
        {
            AZ_Error(
                "ROS2GPSOrientedSensorComponent",
                false,
                "Entity %s does not have a rigid body - stopping GPS sensor.",
                entityId.ToString().c_str());
            StopSensor();
            return;
        }
        else
        {
            const AZ::Quaternion orientationLevel = rigidBody->GetOrientation();
            // determine current orientation relative to GNSS ENU coordinates
            const AZ::Quaternion orientationENU = m_levelRotation * orientationLevel;
            eulerAngles = orientationENU.GetEulerDegreesXYZ();
        }

        // get deviation angle towards east
        float dip = eulerAngles.GetZ();
        // map angle according Sepentrio GNSS: 2. quadrant is [-270°..-180°]
        if(90.0 < dip)
        {
            dip -= 360.0;
        }
        m_gpsOrientedMsg.dip = dip;

        m_gpsOrientedMsg.status.status = gps_msgs::msg::GPSStatus::STATUS_SBAS_FIX;
        GPSOrientedPostProcessingRequestBus::Event(GetEntityId(), &GPSOrientedPostProcessingRequests::ApplyPostProcessing, m_gpsOrientedMsg);

        m_gpsOrientedMsg.header.stamp = ROS2::ROS2ClockInterface::Get()->GetROSTimestamp();
        m_gpsOrientedPublisher->publish(m_gpsOrientedMsg);
    }

} // namespace ROS2Sensors
