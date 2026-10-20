# manip_hardware

Plugin ros2_control do braço (Dynamixel XM540 via SyncRead/SyncWrite) para a bancada STANDALONE: exporta `arm_hardware/ArmHardwareInterface`, citado só por `manip_description/urdf/manip.ros2_control.xacro`.
O robô real carrega `arm_hardware_interface/ArmHWInterface` de `caramelo_hardware` (caramelo_hardware_ws) na Pi; no PC o braço é sempre mock (`use_mock_hardware`).
Correções de driver devem ir para o `caramelo_hardware`; este pacote é cópia e não é fonte de verdade.
