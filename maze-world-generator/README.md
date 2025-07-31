# 🧱 Maze Generator for Gazebo / ROS 2

## 📌 Projektziel

Dieses Projekt generiert ein vollständiges **Labyrinth in SDF-Format** (`.world`), kompatibel mit **Gazebo Harmonic** und **ROS 2 Jazzy**. Die Welt besteht aus modularen Wandsegmenten, wird vollständig per Python erstellt und kann beliebig erweitert werden – z. B. um Rampen, Ziele, Räume oder interaktive Objekte.

## ✅ Aktuell umgesetzt

### ✔️ Maze-Generierung
- Algorithmus: **Recursive Backtracking (Depth-First Search)**.
- Vollständig durchgängiges Maze mit garantiertem Pfad zwischen allen Zellen.

### ✔️ Optimierung der Wandplatzierung
- Wände werden als möglichst große Blöcke (`8m`, `4m`, `2m`, `1m`) gesetzt.
- Reduziert die Anzahl von `<include>`-Einträgen → bessere Simulations-Performance.

### ✔️ Korrekte Weltplatzierung
- Der Maze-Ursprung liegt mittig um den Punkt `(0, 0)`.
- Alle Wand-`<pose>`-Einträge werden korrekt relativ zur Mitte berechnet.

### ✔️ Vollständige `.world`-Datei
- Enthält:
  - Standard-Gazebo-Plugins (Physics, IMU, Scene, UserCommands)
  - Lichtquelle (Sun)
  - Boden (Standard Ground)
  - Alle Wände
- Automatisch erzeugt als `maze_world.world`.

### ✔️ Automatische Außenbegrenzung
- Alle vier Kanten werden mit durchgängigen Wänden eingefasst.
- Verhindert, dass Roboter versehentlich das Maze verlassen.

---

## 📌 Geplante Erweiterungen

| Feature                      | Status     |
|-----------------------------|------------|
| Start- und Zielmarkierung   | ❌ offen    |
| Räume im Inneren            | ❌ offen    |
| Objekte (z. B. Kisten)      | ❌ offen    |
| Rampen / zweite Etage       | ❌ offen    |

## 🧠 Wichtige Designentscheidungen

### 🔁 Maze-Algorithmus
- Verwendet wird **Recursive Backtracking**, auch bekannt als Tiefensuche mit zufälliger Verzweigung.
- Vorteil: Garantiert ein zusammenhängendes Maze ohne isolierte Inseln.

### 🧱 Wandoptimierung

* Lange gerade Wände werden automatisch erkannt und als große Blöcke eingesetzt.
* Dadurch weniger Modelle, geringere Ladezeit, bessere FPS.
* Priorisierte Segmentlängen: `[8, 4, 2, 1]`

### 📐 Positionierung und Offset

* Maze-Zellen beginnen bei `(-maze_width/2, -maze_height/2)`, damit das Labyrinth mittig im Weltursprung liegt.
* Wand-Modelle werden mit korrektem Versatz entlang ihrer Ausrichtung platziert.
* Wandnamen sind eindeutig (`wall_2m_42` etc.) um Konflikte zu vermeiden.

## 🚀 Ausführung

```bash
python3 maze_generator.py
```