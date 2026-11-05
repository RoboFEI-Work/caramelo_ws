#!/usr/bin/env python3
"""Calcula a montagem da camera do punho a partir do calib_camera_record.py.

Modelo: cada tag k esta PARADA em p_k (manip_base_link). Na amostra i,
  p_k = (base_T_flange_i * flange_T_optical * optical_T_tag_i).translacao
Incognitas: flange_T_optical (6) + p_k (3 por tag), minimos quadrados
robustos. Com a montagem certa a mesma tag cai no mesmo ponto vista de
qualquer pose; o RMS antes/depois mostra o quanto a nominal errava.

Saida: o novo <origin> da camera no manipulator_arm.xacro do
caramelo_description (xyz_offset / rpy_offset do rgbd_camera, pai link5).

  python3 calib_camera_solve.py /tmp/calib_camera.jsonl
"""
import argparse
import json
import math
import sys

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation


def mat(r, t):
    m = np.eye(4)
    m[:3, :3] = r
    m[:3, 3] = t
    return m


def rpy_mat(xyz, rpy):
    # URDF: rpy fixos X, Y, Z (R = Rz(yaw) Ry(pitch) Rx(roll)).
    return mat(Rotation.from_euler('xyz', rpy).as_matrix(), xyz)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('jsonl')
    ap.add_argument('--mount-xyz', default='0 -0.055 0.0125',
                    help='xyz_offset ATUAL do rgbd_camera no manipulator_arm.xacro')
    ap.add_argument('--mount-rpy', default='0 -1.57079632679 -1.57079632679',
                    help='rpy_offset ATUAL do rgbd_camera no manipulator_arm.xacro')
    ap.add_argument('--min-per-tag', type=int, default=3)
    args = ap.parse_args()

    nominal = None
    samples = []
    with open(args.jsonl) as fh:
        for line in fh:
            line = line.strip()
            if not line:
                continue
            d = json.loads(line)
            if 'nominal_flange_T_optical' in d:
                nominal = np.array(d['nominal_flange_T_optical'])
            else:
                # Mesma tag em outro trecho (a base andou) = outro ponto fixo.
                key = '%s@%d' % (d['tag'], d.get('segment', 0))
                samples.append((key, np.array(d['base_T_flange']), np.array(d['optical_T_tag'])))
    if nominal is None:
        sys.exit('arquivo sem nominal_flange_T_optical (o gravador nao achou o TF flange -> camera).')

    by_tag = {}
    for s in samples:
        by_tag.setdefault(s[0], []).append(s)
    tags = sorted(t for t, v in by_tag.items() if len(v) >= args.min_per_tag)
    samples = [s for s in samples if s[0] in tags]
    print(f'{len(samples)} amostra(s) em {len(tags)} tag(s): ' +
          ', '.join(f'{t} ({len(by_tag[t])})' for t in tags))
    if len(samples) < 12 or not tags:
        sys.exit('poucas amostras: grave >= 12 (ideal 30+) com o braco em poses bem diferentes.')

    # Diversidade de orientacao do flange (sem ela a rotacao nao e observavel).
    rots = Rotation.from_matrix(np.array([s[1][:3, :3] for s in samples]))
    spread = np.degrees(np.max((rots.mean().inv() * rots).magnitude()))
    print(f'giro maximo do flange entre amostras: {spread:.0f} graus')
    if spread < 20.0:
        print('AVISO: pouca variacao de orientacao (< 20 graus) - resultado pouco confiavel.')

    tag_idx = {t: k for k, t in enumerate(tags)}
    n_t = len(tags)

    def flange_T_opt(x):
        # Correcao pequena sobre a nominal: X = nominal * delta.
        delta = mat(Rotation.from_rotvec(x[:3]).as_matrix(), x[3:6])
        return nominal @ delta

    def points(x6):
        X = flange_T_opt(x6)
        return np.array([(s[1] @ X @ s[2])[:3, 3] for s in samples])

    def residuals(x):
        pts = points(x[:6])
        p = x[6:].reshape(n_t, 3)
        return (pts - p[[tag_idx[s[0]] for s in samples]]).ravel()

    def report(label, x6):
        pts = points(x6)
        out = []
        for t in tags:
            ids = [i for i, s in enumerate(samples) if s[0] == t]
            c = pts[ids].mean(axis=0)
            err = np.linalg.norm(pts[ids] - c, axis=1)
            out.append(err)
            print(f'  {label} {t}: espalhamento max {err.max() * 1000:.1f} mm, rms '
                  f'{math.sqrt((err ** 2).mean()) * 1000:.1f} mm, z medio {c[2]:+.4f}')
        allerr = np.concatenate(out)
        print(f'  {label} TOTAL: rms {math.sqrt((allerr ** 2).mean()) * 1000:.1f} mm, '
              f'max {allerr.max() * 1000:.1f} mm')
        return pts

    print('\nCOM A MONTAGEM NOMINAL (a mesma tag vista de poses diferentes):')
    pts0 = report('nominal', np.zeros(6))
    x0 = np.zeros(6 + 3 * n_t)
    for t in tags:
        ids = [i for i, s in enumerate(samples) if s[0] == t]
        x0[6 + 3 * tag_idx[t]:9 + 3 * tag_idx[t]] = pts0[ids].mean(axis=0)
    sol = least_squares(residuals, x0, loss='soft_l1', f_scale=0.005)
    print('\nCOM A MONTAGEM CALIBRADA:')
    report('calibrada', sol.x[:6])

    d_ang = np.degrees(np.linalg.norm(sol.x[:3]))
    d_rpy = np.degrees(Rotation.from_rotvec(sol.x[:3]).as_euler('xyz'))
    print(f'\ncorrecao no frame optico: giro {d_ang:.2f} graus (rpy {d_rpy[0]:+.2f} {d_rpy[1]:+.2f} '
          f'{d_rpy[2]:+.2f}), deslocamento {sol.x[3] * 1000:+.1f} {sol.x[4] * 1000:+.1f} '
          f'{sol.x[5] * 1000:+.1f} mm')

    # flange_T_optical = M * C (C = parte interna da D455, fixa):
    # M_novo = X_novo * X_nominal^-1 * M_nominal.
    m_nom = rpy_mat([float(v) for v in args.mount_xyz.split()],
                    [float(v) for v in args.mount_rpy.split()])
    m_new = flange_T_opt(sol.x[:6]) @ np.linalg.inv(nominal) @ m_nom
    xyz = m_new[:3, 3]
    rpy = Rotation.from_matrix(m_new[:3, :3]).as_euler('xyz')
    print('\nNOVO <origin> da camera (caramelo_description/urdf/mech/manipulator_arm.xacro, rgbd_camera):')
    print(f'    xyz_offset="{xyz[0]:.4f} {xyz[1]:.4f} {xyz[2]:.4f}"')
    print(f'    rpy_offset="{rpy[0]:.5f} {rpy[1]:.5f} {rpy[2]:.5f}"')
    if d_ang > 10.0 or np.linalg.norm(sol.x[3:6]) > 0.03:
        print('AVISO: correcao grande (> 10 graus ou > 3 cm) - confira as amostras antes de aplicar.')


if __name__ == '__main__':
    main()
