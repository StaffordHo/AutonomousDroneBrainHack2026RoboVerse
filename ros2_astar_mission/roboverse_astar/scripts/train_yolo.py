#!/usr/bin/env python3
import argparse

from ultralytics import YOLO


def main():
    parser = argparse.ArgumentParser(description="Train YOLO for RoboVerse red/yellow fuel barrels.")
    parser.add_argument("--data", default="/home/stafford99/roboverse_qualifier/ros2_astar_mission/datasets/fuel_barrels_v1/data.yaml")
    parser.add_argument("--model", default="yolov8n.pt")
    parser.add_argument("--epochs", type=int, default=100)
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--batch", type=int, default=8)
    parser.add_argument("--project", default="/home/stafford99/roboverse_qualifier/ros2_astar_mission/training_runs")
    parser.add_argument("--name", default="fuel_barrels_yolo")
    args = parser.parse_args()

    model = YOLO(args.model)
    model.train(
        data=args.data,
        epochs=args.epochs,
        imgsz=args.imgsz,
        batch=args.batch,
        project=args.project,
        name=args.name,
        patience=30,
        hsv_h=0.035,
        hsv_s=0.65,
        hsv_v=0.55,
        degrees=8.0,
        translate=0.12,
        scale=0.55,
        shear=2.0,
        perspective=0.0008,
        fliplr=0.5,
        mosaic=1.0,
        mixup=0.08,
        copy_paste=0.10,
        close_mosaic=12,
    )


if __name__ == "__main__":
    main()
