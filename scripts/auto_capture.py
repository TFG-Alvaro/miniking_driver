#!/usr/bin/env python3
import itertools
import os
import signal
import subprocess
import time
import yaml
import rospkg

def main():
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('miniking_driver')
    yaml_path = os.path.join(pkg_path, 'config', 'sonar_params.yaml')
    bags_dir = os.path.join(pkg_path, 'bags')
    os.makedirs(bags_dir, exist_ok=True)

    # Arrays de parámetros clave para detección de objetos en el fondo de la piscina
    gains = [35, 50, 65]             # Sensibilidad frente a rebotes del suelo
    ranges = [2, 5]                  # Escala en metros (2 m = máximo detalle radial)
    resolutions = ["Medium", "High"] # Paso angular (velocidad vs nitidez del objeto)
    ad_lows = [20, 45]               # Umbral mínimo para limpiar ruido de reverberación
    duracion_ensayo = 15             # Segundos de grabación rosbag por ensayo

    with open(yaml_path, 'r') as f:
        base_params = yaml.safe_load(f)

    combinaciones = list(itertools.product(gains, ranges, resolutions, ad_lows))
    total = len(combinaciones)
    print(f"=== INICIANDO CAMPAÑA PISCINA (SONAR BOCA ABAJO): {total} ENSAYOS ===")

    for idx, (gain, rng, res, ad_low) in enumerate(combinaciones, start=1):
        nombre_ensayo = f"piscina_{idx:02d}_g{gain}_r{rng}_{res}_ad{ad_low}"
        bag_file = os.path.join(bags_dir, f"{nombre_ensayo}.bag")

        # 1. Actualizar parámetros en sonar_params.yaml
        params = base_params.copy()
        params['ensayo'] = nombre_ensayo
        params['duracion'] = 0
        params['gain'] = gain
        params['range_scale'] = rng
        params['resolution'] = res
        params['ad_low'] = ad_low
        params['inverted'] = True  # Montaje boca abajo apuntando al fondo

        with open(yaml_path, 'w') as f:
            yaml.safe_dump(params, f, default_flow_style=False)

        print(f"\n[{idx}/{total}] Lanzando {nombre_ensayo} ({duracion_ensayo} s)...")

        # 2. Lanzar sonar.launch con rosbag record (sin abrir ventana RViz)
        cmd = [
            "roslaunch", "miniking_driver", "sonar.launch",
            "rviz:=false",
            "record_bag:=true",
            f"bag_prefix:={nombre_ensayo}"
        ]
        proceso = subprocess.Popen(cmd, preexec_fn=os.setsid)

        try:
            time.sleep(duracion_ensayo)
        except KeyboardInterrupt:
            print("\nCampaña interrumpida manualmente.")
            os.killpg(os.getpgid(proceso.pid), signal.SIGINT)
            proceso.wait()
            break

        # 3. Detener roslaunch y cerrar el .bag limpiamente
        print(f"[{idx}/{total}] Guardado {bag_file}")
        os.killpg(os.getpgid(proceso.pid), signal.SIGINT)
        proceso.wait()
        time.sleep(2.0)  # Pausa para liberar /dev/ttyUSB0 entre ensayos

    print("\n=== CAMPAÑA FINALIZADA ===")

if __name__ == '__main__':
    main()
