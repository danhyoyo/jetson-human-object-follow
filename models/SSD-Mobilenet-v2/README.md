# SSD-MobileNet-v2 model assets

This directory contains the portable model assets used by person tracking:

- `ssd_mobilenet_v2_coco.uff` — 90-class MS-COCO SSD-MobileNet-v2 network
- `ssd_coco_labels.txt` — class labels containing `person`
- `ssd_mobilenet_v2_coco.uff.sha256sum` — expected UFF SHA-256

The TensorRT `.engine` file is intentionally not packaged because it is a
generated cache tied to the JetPack/TensorRT/GPU version of a particular
Jetson.  `setup_jetson.sh` installs the portable assets into jetson-inference's
network directory when needed.  The first person-mode start on a new Jetson
can take several minutes while TensorRT builds its local engine.
