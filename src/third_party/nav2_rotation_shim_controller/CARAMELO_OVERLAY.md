# Overlay do nav2_rotation_shim_controller 1.3.13 (Caramelo, 2026-11-13)

Copia do pacote da tag 1.3.13 do Nav2 (a mesma versao do apt em /opt/ros/jazzy),
compilada no workspace para sobrepor a do sistema. Unica mudanca de codigo:
com `use_path_orientations: true`, o ponto amostrado do plano e' transformado
para o frame da pose do robo antes de comparar os yaws
(`src/nav2_rotation_shim_controller.cpp`, marcado CARAMELO). No 1.3.13 o yaw do
plano (map) era comparado com o do robo (odom) sem transformar; com o map->odom
girado (157-180 graus nas corridas de 13/11) o shim giraria para o lado errado.
O Nav2 posterior ao Jazzy ja recebe o plano transformado.

Teste: `test/test_shim_controller.cpp`, `pathOrientationsUseControllerFrame`.
Reverter: apagar esta pasta (e `build/`, `install/` do pacote) e recompilar: volta
a valer o pacote do /opt/ros/jazzy. Ao atualizar o Nav2 do apt, conferir se a
versao mudou e se este overlay ainda e' necessario.
