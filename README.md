# ft_kalman

Filtre de Kalman (Rust, sans dépendance) qui suit la trajectoire d'un véhicule à partir des flux
bruités de `imu-sensor-stream`.

## Utilisation

```sh
make                                   # binaire statique ./ft_kalman (RUSTFLAGS="-D warnings")
./imu-sensor-stream-linux -s 42 -d 42 -p 4242 &
./ft_kalman -p 4242                     # -h pour l'aide
```

| Option | Effet |
|---|---|
| `-p, --port` / `-a, --address` | où écoute le flux (défaut `127.0.0.1:4242`) |
| `-m, --meanspeed` (`--filterspeed`) | temps de réponse moyen / pire du filtre |
| `-d, --delta` | erreur vs position vraie (flux lancé avec `--debug`) |
| `-v, --visual <f.html>` | visualisateur 3D (rotation souris) + 2D, GPS, trajectoire vraie, variance 1σ |

Codes de sortie : `0` fin normale, `1` arrêt propre sur erreur (flux muet / absent / erreur du flux /
filtre qui n'est plus fiable), `2` mauvais arguments. Jamais de `panic`.

## Le modèle

* État `x = [px py pz vx vy vz]`.
* **Prédiction (100 Hz)** : l'accéléromètre (déjà dans le repère XYZ) est l'entrée de commande :
  `p += v·dt`, `v += a·dt` (même schéma d'intégration que le flux).
* **Mise à jour GPS (toutes les 3 s)** : mesure directe de `p`.
* **Mise à jour « cap » (100 Hz)** : le véhicule ne se déplace que selon son axe longitudinal, donc la
  vitesse est colinéaire à l'axe avant donné par les angles d'Euler (ZYX : roll, pitch, yaw). Les
  composantes de la vitesse orthogonales à ce cap sont une mesure (EKF sur l'angle, base orthogonale à
  la direction *estimée* pour que la correction soit purement transverse). Le gyroscope n'est donc pas
  jeté après l'init : il rend la vitesse précise en continu.
* Mise à jour de la covariance en **forme de Joseph** + symétrisation → stable sur 90 min (540 000 pas).

## Bonus

* **Visualisateur** (`--visual`) : page HTML autonome, 3D orientable, vue de dessus, variance dessinée.
* **Rapidité** : ~0,013 ms de réponse moyenne (`--meanspeed`), le simulateur est le facteur limitant.
* **Plus de bruit** : les écarts-types de l'accéléromètre, du gyroscope et du GPS sont **estimés en
  ligne** (différences successives pour les capteurs rapides, innovation pour le GPS) ; aucune valeur
  n'est codée en dur au-delà d'un a priori. Mesuré avec les options `-n`, `-a`, `-g` du flux.
* **Fonctionnalités en plus** : fusion du cap (ci-dessus), estimation adaptative des bruits, détection de
  divergence (arrêt propre si l'incertitude de position dépasse 5 m ou si une valeur n'est plus finie).

## Observations sur le flux (utiles à l'évaluation)

* Les sigmas affichés dans le sujet sont des valeurs de base ; le flux sort environ
  accéléromètre 0,01 · gyroscope 0,01 · GPS 1,0 (mesuré, `-n` multiplie accéléromètre et GPS).
* Le GPS (`POSITION`) n'est **pas** envoyé en mode `--debug`.
* Limite théorique d'un tel filtre : σ_position ≈ 0,32 m × (facteur de bruit). Au-delà de ×4–×5 des
  dépassements de 5 m deviennent inévitables, quelle que soit l'implémentation.

## Tests

```sh
make test      # = cargo test --release
```

* `matrix`, `vector`, `noise`, `utils`, `kalman` : tests unitaires (dont tous les cas d'erreur).
* `simulator` : faux flux UDP (dialogue complet, timeout, erreur serveur, données corrompues).
* `tracker` : trajectoires synthétiques (90 min, bruit ×2/×3, moins de bruit, trames bizarres, divergence).
* `cli` : le binaire contre un flux synthétique (+ options invalides, pas de serveur).
* `e2e` : le vrai `imu-sensor-stream` (plusieurs seeds, 15 min, bruit, `--debug`, flux tué en route).
