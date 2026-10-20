# manip_description

Bancada STANDALONE do braço sozinho (raiz `base_link`): URDF/xacro e meshes usados só por `manip_moveit_config/config/manip.urdf.xacro` (move_group.launch.py / Setup Assistant).
O robô real usa `caramelo_description` (caramelo_hardware_ws), que tem os mesmos STL e o xacro do robô combinado; o `manip_stack.launch.py` nunca carrega este pacote.
Edições de geometria do braço devem ir para o `caramelo_description`; este pacote não é fonte de verdade.
