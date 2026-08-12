import os
from glob import glob

from setuptools import find_packages, setup

package_name = "caramelo_web"


def arquivos_do_frontend():
    """Instala www/ inteiro, preservando subpastas.

    O frontend e' VERSIONADO ja' pronto para servir: em competicao nao ha'
    internet, entao nao pode existir 'npm install' nem CDN no caminho.
    """
    saida = []
    for raiz, _dirs, nomes in os.walk("www"):
        if not nomes:
            continue
        destino = os.path.join("share", package_name, raiz)
        saida.append((destino, [os.path.join(raiz, n) for n in nomes]))
    return saida


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ] + arquivos_do_frontend(),
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="RoboFEI@Work",
    maintainer_email="victoroliveiraayres@gmail.com",
    description="Painel web de engenharia do Caramelo.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "caramelo_web_server = caramelo_web.server:main",
        ],
    },
)
