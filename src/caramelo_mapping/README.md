# caramelo_mapping

Pacote de mapeamento do Caramelo no PC. O fluxo principal usa
`slam_toolbox` via:

```bash
ros2 launch caramelo_mapping slam.launch.py map_name:=sala_520
```

Durante o SLAM, o launch tambem pode publicar markers das Service Areas em:

```text
/caramelo/service_areas/markers
```

Para um mapa novo, use o fluxo hibrido:

1. mapear com SLAM;
2. salvar poses provisorias de START/WS/SH/PP/RT se ajudar;
3. salvar `map.yaml` e `map.pgm`;
4. reabrir o mapa com Nav2/AMCL;
5. sobrescrever as poses finais para docking preciso.

Tutorial completo:

```text
docs/service_areas_docking.md
```
