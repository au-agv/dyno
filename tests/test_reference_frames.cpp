    /*
    SPDLOG_INFO("Position using GetPos(), as recorded: {}\nPosition using
    COMFrame->GetPos(): {}\nPosition using " "AuxFrame->GetPos(): {}",
                vehicle_->GetVehicle()->GetPos().x(),
                vehicle_->GetVehicle()->GetChassisBody()->GetFrameCOMToAbs().GetPos().x(),
                vehicle_->GetVehicle()->GetChassisBody()->GetFrameRefToAbs().GetPos().x());

    SPDLOG_INFO("Velocity using GetVel(), as recorded: {}\nVelocity using
    COMFrame->GetVel(): {}\nVelocity using " "AuxFrame->GetVel(): {}",
                vehicle_->GetVehicle()->GetChassisBody()->GetPosDt().x(),
                vehicle_->GetVehicle()->GetChassisBody()->GetFrameCOMToAbs().GetPosDt().x(),
                vehicle_->GetVehicle()->GetChassisBody()->GetFrameRefToAbs().GetPosDt().x());

    SPDLOG_INFO("Angular velocity using GetRot(), as recorded: {}\nAngular
    velocity using COMFrame->GetRot(): "
                "{}\nAngular velocity using "
                "AuxFrame->GetRot(): {}",
                vehicle_->GetVehicle()->GetChassisBody()->GetRotDt().GetCardanAnglesXYZ().x(),
                vehicle_->GetVehicle()->GetChassisBody()->GetFrameCOMToAbs().GetRotDt().GetCardanAnglesXYZ().x(),
                vehicle_->GetVehicle()->GetChassisBody()->GetFrameRefToAbs().GetRotDt().GetCardanAnglesXYZ().x());

    SPDLOG_INFO(
        "Acceleration using GetAcc(), as recorded: {}\nPosition using
    COMFrame->GetAcc(): {}\nAcceleration using " "AuxFrame->GetAcc(): {}",
        vehicle_->GetVehicle()->GetChassisBody()->GetPosDt2().x(),
        vehicle_->GetVehicle()->GetChassisBody()->GetFrameCOMToAbs().GetPosDt2().x(),
        vehicle_->GetVehicle()->GetChassisBody()->GetFrameRefToAbs().GetPosDt2().x());

    SPDLOG_INFO("Angular acceleration using GetRot(), as recorded: {}\nAngular
    acceleration using COMFrame->GetRot(): "
                "{}\nAngular acceleration using "
                "AuxFrame->GetRot(): {}",
                vehicle_->GetVehicle()->GetChassisBody()->GetRotDt2().GetCardanAnglesXYZ().x(),
                vehicle_->GetVehicle()->GetChassisBody()->GetFrameCOMToAbs().GetRotDt2().GetCardanAnglesXYZ().x(),
                vehicle_->GetVehicle()->GetChassisBody()->GetFrameRefToAbs().GetRotDt2().GetCardanAnglesXYZ().x());
*/