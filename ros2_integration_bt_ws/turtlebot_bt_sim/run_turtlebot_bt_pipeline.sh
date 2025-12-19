#!/usr/bin/env bash
set -eo pipefail

log() {
  echo "[$(date +'%Y-%m-%d %H:%M:%S')] [INFO] $*"
}

err() {
  echo "[$(date +'%Y-%m-%d %H:%M:%S')] [ERROR] $*" >&2
}

# PROMPT="${1:-}"
AUTO_MODE=0
PROMPT=""

while [ "$#" -gt 0 ]; do
  case "$1" in
    --auto)
      AUTO_MODE=1
      shift
      ;;
    *)
      # Tout le reste est considéré comme le prompt (souvent une seule chaîne entre guillemets)
      PROMPT="$*"
      break
      ;;
  esac
done

if [ -z "$PROMPT" ]; then
  read -rp "Description de la mission (prompt LLM) : " PROMPT
fi

# Chemins par défaut (surchargeables via variables d'environnement)
ROS_MAIN_WS="${ROS_MAIN_WS:-$HOME/ros2_ws}"
BT_WS="${BT_WS:-$HOME/studies/dev/nav4rails/BT_Tutorials/ros2_integration_bt_ws}"

log "Sourcing ROS 2 Humble et workspace principal ($ROS_MAIN_WS)..."
source /opt/ros/humble/setup.bash
source "$ROS_MAIN_WS/install/setup.bash"

cd "$BT_WS"
log "Sourcing workspace BT ($BT_WS)..."
if [ -f "install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source install/setup.bash
else
  log "Aucun install/setup.bash trouvé, le workspace sera compilé plus tard."
fi

cd "$BT_WS/turtlebot_bt_sim"

if [ ! -f "gen_turtlebot_bt.py" ]; then
  err "gen_turtlebot_bt.py introuvable dans $BT_WS/turtlebot_bt_sim"
  exit 1
fi

log "Génération du Behavior Tree via LLM..."
python3 gen_turtlebot_bt.py --prompt "$PROMPT"

LATEST_XML="$(ls -1t trees/turtlebot_mission_generated_*.xml 2>/dev/null | head -n 1 || true)"

if [ -z "$LATEST_XML" ] || [ ! -f "$LATEST_XML" ]; then
  err "Aucun fichier généré trouvé dans trees/turtlebot_mission_generated_*.xml"
  exit 1
fi

log "Dernier Behavior Tree généré : $LATEST_XML"

if [ -f trees/turtlebot_mission.xml ]; then
  BACKUP="trees/turtlebot_mission_backup_$(date +'%Y%m%d%H%M%S').xml"
  log "Backup de trees/turtlebot_mission.xml vers $BACKUP"
  cp trees/turtlebot_mission.xml "$BACKUP"
else
  log "Pas de fichier trees/turtlebot_mission.xml existant, pas de backup créé."
fi

log "Aperçu du Behavior Tree généré ($LATEST_XML) :"
echo "------------------------------------------------------------"
cat "$LATEST_XML"
echo
echo "------------------------------------------------------------"
if [ "$AUTO_MODE" -eq 0 ]; then
  while true; do
    read -r -p "Valider ce Behavior Tree et remplacer trees/turtlebot_mission.xml ? [O/n] " REPLY
    # Normalisation en minuscules
    ANSWER="${REPLY,,}"
    case "$ANSWER" in
      ""|"o"|"oui"|"y"|"yes")
        log "Validation acceptée, poursuite du pipeline."
        break
        ;;
      "n"|"non"|"no")
        log "Remplacement annulé par l'utilisateur. Arrêt du pipeline."
        exit 0
        ;;
      *)
        echo "Réponse non reconnue. Merci de répondre par Oui ou Non."
        ;;
    esac
  done
else
  log "Mode automatique (--auto) activé, étape de validation utilisateur ignorée."
fi

log "Remplacement de trees/turtlebot_mission.xml par $LATEST_XML"
cp "$LATEST_XML" trees/turtlebot_mission.xml

cd "$BT_WS"
log "Compilation de turtlebot_bt_sim..."
colcon build --packages-select turtlebot_bt_sim

log "Sourcing workspace BT après compilation..."
# shellcheck disable=SC1091
source install/setup.bash

export TURTLEBOT3_MODEL="${TURTLEBOT3_MODEL:-burger}"
log "TURTLEBOT3_MODEL=$TURTLEBOT3_MODEL"

log "Lancement de Gazebo + Behavior Tree (ros2 launch turtlebot_bt_sim turtlebot_bt_sim.launch.py)..."
ros2 launch turtlebot_bt_sim turtlebot_bt_sim.launch.py


