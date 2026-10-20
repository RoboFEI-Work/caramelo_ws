# caramelo_localization

AMCL + map_server do Caramelo (`ros2 launch caramelo_localization global_localization.launch.py map_name:=<mapa>`), incluído pelo `navigation.launch.py`; com `use_keepout:=true` sobe também os servidores do Keepout Filter (`config/keepout_filter.yaml`, máscara em `maps/<mapa>/keepout_mask.yaml`).
Configs: `config/amcl.yaml` (OmniMotionModel) e `rviz/global_localization.rviz` (RViz padrão do bringup).
