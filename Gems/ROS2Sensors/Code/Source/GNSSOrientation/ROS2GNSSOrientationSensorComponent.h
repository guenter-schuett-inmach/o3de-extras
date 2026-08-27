/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/Math/Transform.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <ROS2/Sensor/Events/TickBasedSource.h>
#include <ROS2/Sensor/ROS2SensorComponentBase.h>
#include <ROS2Sensors/ROS2SensorsTypeIds.h>
#include <rclcpp/publisher.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>

namespace ROS2Sensors
{
    //! Global Navigation Satellite Systems (GNSS) Orientation sensor component class
    //! It provides GPSFix data of sensor's position and orientation in GNSS frame which is defined by GNSS origin offset
    //! Offset is provided as latitude [deg], longitude [deg], altitude [m] of o3de global frame
    //! Orientation is provided as dip angle [°]
    //! It is assumed that o3de global frame overlaps with ENU coordinate system
    class ROS2GNSSOrientationSensorComponent : public ROS2::ROS2SensorComponentBase<ROS2::TickBasedSource>
    {
    public:
        using SensorBaseType = ROS2::ROS2SensorComponentBase<ROS2::TickBasedSource>;

        AZ_COMPONENT(ROS2GNSSOrientationSensorComponent, ROS2GNSSOrientationSensorComponentTypeId, SensorBaseType);
        ROS2GNSSOrientationSensorComponent();
        ROS2GNSSOrientationSensorComponent(const ROS2::SensorConfiguration& sensorConfiguration);
        ~ROS2GNSSOrientationSensorComponent() = default;
        static void Reflect(AZ::ReflectContext* context);

        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        //////////////////////////////////////////////////////////////////////////
        // Component overrides
        void Activate() override;
        void Deactivate() override;
        //////////////////////////////////////////////////////////////////////////

    private:
        ///! Requests gnss message publication.
        void FrequencyTick();

        std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::QuaternionStamped>> m_GNSSOrientationPublisher;
        geometry_msgs::msg::QuaternionStamped m_GNSSOrientationMsg;

        AZ::Quaternion m_levelRotation;
    };

} // namespace ROS2Sensors
