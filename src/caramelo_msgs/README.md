# caramelo_msgs

Interfaces próprias do Caramelo: actions `AlignToDock` e `NudgeBase` (servidas pelo `dock_align_node`; clientes GoToWS/amt1_sort/GUI) e `SaveServiceAreaPose`, mais os services `InitServiceAreas`, `ListServiceAreas` e `ValidateServiceAreas` (servidos pelo `service_area_manager_node`; cliente GUI).
Compilar antes dos demais pacotes: `colcon build --packages-select caramelo_msgs`.
