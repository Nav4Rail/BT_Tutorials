#!/usr/bin/env python3

"""
CLI de génération de Behavior Tree pour `turtlebot_bt_sim`.

Principe :
- Reçoit un prompt en langage naturel décrivant la mission du TurtleBot.
- Appelle un LLM (par ex. Mistral, interface OpenAI-compatible) pour générer
  une description structurée des étapes.
- Construit un arbre XML `turtlebot_mission.xml` STRICTEMENT compatible avec
  les nœuds BT implémentés dans ce package :
    - Idle(duration)
    - DriveForward(duration, speed)
    - Rotate(duration, angular_speed)
    - StopRobot()

Le LLM est donc contraint à ne renvoyer qu'une structure simple (JSON),
et la conformité XML est garantie par ce script.

Usage (exemple) :

  cd ros2_integration_bt_ws/turtlebot_bt_sim
  python3 gen_turtlebot_bt.py --prompt "
    Attendre 5s, avancer 2m, tourner à gauche,
    avancer encore, puis s'arrêter."

Variables d'environnement possibles :
- LLM_API_KEY   : clé API (obligatoire si --api-key non fourni)
- LLM_MODEL     : nom du modèle (ex : mistral-bt-generator)
- LLM_API_BASE  : URL de base compatible OpenAI (facultatif, ex Mistral)

Remarque : ce script n'impose pas un fournisseur spécifique, il suppose
une API de type OpenAI (p. ex. Mistral, OpenAI, etc.).
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional
from xml.etree.ElementTree import (
    Element,
    SubElement,
    Comment,
    ElementTree,
)

import datetime

DEFAULT_MODEL = os.getenv("LLM_MODEL", "mistral-medium")
DEFAULT_XML_PATH = (
    Path(__file__).resolve().parent / "trees" / f"turtlebot_mission_generated_{datetime.datetime.now().strftime('%Y%m%d%H%M%S')}.xml"
)


# ---------------------------------------------------------------------------
# Modèle de données intermédiaire : ce que doit produire le LLM
# ---------------------------------------------------------------------------


@dataclass
class BtStep:
    """Étape élémentaire de la mission TurtleBot."""

    action: str  # Idle | DriveForward | Rotate | StopRobot
    duration: Optional[float] = None
    speed: Optional[float] = None
    angular_speed: Optional[float] = None
    comment: Optional[str] = None


ALLOWED_ACTIONS = {"Idle", "DriveForward", "Rotate", "StopRobot"}


def _strip_markdown_fences(raw: str) -> str:
    """
    Certains LLM renvoient le JSON entouré de balises de code Markdown,
    par exemple:

    ```json
    [ ... ]
    ```

    Cette fonction enlève ces balises éventuelles pour ne garder
    que le JSON brut.
    """
    text = raw.strip()
    if text.startswith("```"):
        lines = text.splitlines()
        # Retirer la première ligne si c'est une fence ``` ou ```json
        if lines and lines[0].lstrip().startswith("```"):
            lines = lines[1:]
        # Retirer la dernière ligne si c'est une fence ```
        if lines and lines[-1].strip().startswith("```"):
            lines = lines[:-1]
        text = "\n".join(lines).strip()
    return text


def _parse_steps_from_llm_output(raw: str) -> List[BtStep]:
    """
    Parse la sortie brute du LLM.

    Le LLM est fine-tuned / prompté pour renvoyer UNIQUEMENT
    un JSON de la forme :

      [
        {
          "action": "Idle",
          "duration": 10.0,
          "comment": "Phase d'attente avant de démarrer la mission"
        },
        {
          "action": "DriveForward",
          "duration": 5.0,
          "speed": 0.2,
          "comment": "Avance tout droit"
        },
        ...
      ]
    """
    cleaned = _strip_markdown_fences(raw)
    try:
        data = json.loads(cleaned)
    except Exception as exc:  # pragma: no cover
        raise RuntimeError(
            f"Réponse LLM non JSON.\nErreur: {exc}\nContenu:\n{raw}"
        ) from exc

    if not isinstance(data, list):
        raise ValueError("La sortie LLM doit être une liste JSON d'étapes.")

    steps: List[BtStep] = []
    for item in data:
        if not isinstance(item, dict):
            raise ValueError(
                "Entrée d'étape invalide (doit être un objet JSON) : "
                f"{item}"
            )

        action = item.get("action")
        if action not in ALLOWED_ACTIONS:
            allowed = ", ".join(sorted(ALLOWED_ACTIONS))
            raise ValueError(f"Action '{action}' non supportée. "
                             f"Actions possibles : {allowed}")

        step = BtStep(
            action=action,
            duration=item.get("duration"),
            speed=item.get("speed"),
            angular_speed=item.get("angular_speed"),
            comment=item.get("comment"),
        )
        steps.append(step)

    if not steps:
        raise ValueError("La liste d'étapes générée par le LLM est vide.")

    return steps


# ---------------------------------------------------------------------------
# Appel LLM (interface OpenAI-compatible pour Mistral ou autre)
# ---------------------------------------------------------------------------


def call_llm_for_bt_steps(
    natural_language_prompt: str,
    api_key: str,
    model: str = DEFAULT_MODEL,
    api_base: Optional[str] = None,
) -> List[BtStep]:
    """
    Appelle un modèle LLM (ex : Mistral) pour obtenir
    la description des étapes.

    Cette fonction suppose une API compatible OpenAI (client 'openai' v1).
    Pour Mistral, configurez `LLM_API_BASE` / `LLM_MODEL`
    selon votre déploiement.
    """
    print(
        f"Calling LLM for BT steps with model: {model} "
        f"and api_base: {api_base}"
    )
    print(f"API key: {api_key}")
    print(f"Natural language prompt: {natural_language_prompt}")
    try:
        from openai import OpenAI  # type: ignore
    except ImportError as exc:  # pragma: no cover
        msg = (
            "Le package 'openai' est requis pour appeler le LLM.\n"
            "Installez-le par exemple avec :\n"
            "  pip install openai"
        )
        raise RuntimeError(msg) from exc

    client_kwargs: Dict[str, Any] = {"api_key": api_key}
    if api_base:
        client_kwargs["base_url"] = api_base
    client = OpenAI(**client_kwargs)

    system_prompt = (
        "Tu es un assistant spécialisé en Behavior Trees pour TurtleBot3.\n"
        "Ta tâche est de convertir un objectif de mission en une liste\n"
        "JSON d'étapes utilisant UNIQUEMENT les actions suivantes : "
        "Idle, DriveForward, Rotate, StopRobot.\n"
        "\n"
        "Règles STRICTES de sortie :\n"
        "- La sortie doit être UNIQUEMENT un JSON valide "
        "(aucun texte autour).\n"
        "- Le JSON est un tableau d'objets avec les clés possibles :\n"
        "  - 'action' (obligatoire) : "
        "'Idle' | 'DriveForward' | 'Rotate' | 'StopRobot'\n"
        "  - 'duration' (optionnelle, float en secondes)\n"
        "  - 'speed' (optionnelle, float en m/s, pour DriveForward)\n"
        "  - 'angular_speed' (optionnelle, float en rad/s, "
        "pour Rotate)\n"
        "  - 'comment' (optionnelle, courte description textuelle "
        "en français)\n"
        "\n"
        "Exemple de sortie JSON :\n"
        "[\n"
        "  {\"action\": \"Idle\", \"duration\": 10.0, "
        "\"comment\": \"Phase d'attente\"},\n"
        "  {\"action\": \"DriveForward\", \"duration\": 5.0, "
        "\"speed\": 0.2, \"comment\": \"Avance tout droit\"},\n"
        "  {\"action\": \"Rotate\", \"duration\": 4.0, "
        "\"angular_speed\": 0.6, \"comment\": \"Tourne sur place\"},\n"
        "  {\"action\": \"DriveForward\", \"duration\": 3.0, "
        "\"speed\": 0.2, \"comment\": \"Avance à nouveau\"},\n"
        "  {\"action\": \"StopRobot\", \"comment\": \"Arrêt de sécurité\"}\n"
        "]\n"
    )

    completion = client.chat.completions.create(
        model=model,
        temperature=0.2,
        messages=[
            {"role": "system", "content": system_prompt},
            {
                "role": "user",
                "content": (
                    "Objectif de mission (langage naturel) :\n"
                    f"{natural_language_prompt}\n\n"
                    "Renvoie UNIQUEMENT le JSON d'étapes, sans explication."
                ),
            },
        ],
    )

    content = completion.choices[0].message.content or ""
    return _parse_steps_from_llm_output(content.strip())


# ---------------------------------------------------------------------------
# Construction du XML BehaviorTree.CPP
# ---------------------------------------------------------------------------


def _indent_xml(elem: Element, level: int = 0) -> None:
    """
    Indente l'arbre XML pour produire une sortie lisible,
    sans en-tête XML, et proche de l'exemple fourni.
    """
    indent_str = "  "
    i = "\n" + level * indent_str
    if len(elem):
        if not elem.text or not elem.text.strip():
            elem.text = i + indent_str
        for child in elem:
            _indent_xml(child, level + 1)
        if not child.tail or not child.tail.strip():
            child.tail = i  # type: ignore[name-defined]
    if level and (not elem.tail or not elem.tail.strip()):
        elem.tail = i


def build_bt_xml(steps: List[BtStep]) -> ElementTree:
    """
    Construit l'arbre XML complet à partir de la liste d'étapes.

    Schéma imposé :

    <root BTCPP_format="4" main_tree_to_execute="MainTree">
      <BehaviorTree ID="MainTree">
        <Sequence name="MissionSequence">
          <!-- Commentaire optionnel -->
          <Action ID="..."/>
          ...
        </Sequence>
      </BehaviorTree>
    </root>
    """
    if not steps:
        raise ValueError(
            "Aucune étape fournie pour construire le Behavior Tree."
        )

    root = Element("root", BTCPP_format="4", main_tree_to_execute="MainTree")
    bt = SubElement(root, "BehaviorTree", ID="MainTree")
    seq = SubElement(bt, "Sequence", name="MissionSequence")

    for step in steps:
        if step.comment:
            seq.append(Comment(f" {step.comment} "))

        attrs: Dict[str, str] = {"ID": step.action}

        # Gestion des ports en fonction de l'action
        if step.duration is not None:
            attrs["duration"] = f"{float(step.duration):.1f}"
        if step.action == "DriveForward" and step.speed is not None:
            attrs["speed"] = f"{float(step.speed):.1f}"
        if step.action == "Rotate" and step.angular_speed is not None:
            attrs["angular_speed"] = f"{float(step.angular_speed):.1f}"

        SubElement(seq, "Action", **attrs)

    tree = ElementTree(root)
    _indent_xml(root)
    return tree


def write_bt_xml(tree: ElementTree, output_path: Path) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    # Pas de déclaration XML en tête, pour coller à l'exemple fourni.
    tree.write(output_path, encoding="utf-8", xml_declaration=False)


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


def parse_args(argv: Optional[List[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Générateur de Behavior Tree (turtlebot_mission.xml) "
            "pour turtlebot_bt_sim, basé sur un modèle LLM "
            "(ex : Mistral)."
        )
    )
    parser.add_argument(
        "--prompt",
        "-p",
        type=str,
        help="Description en langage naturel de la mission. "
        "Si omis, le prompt est lu sur stdin.",
    )
    parser.add_argument(
        "--output",
        "-o",
        type=str,
        default=str(DEFAULT_XML_PATH),
        help=(
            "Chemin du fichier XML de sortie "
            f"(par défaut: {DEFAULT_XML_PATH})."
        ),
    )
    parser.add_argument(
        "--model",
        "-m",
        type=str,
        default=DEFAULT_MODEL,
        help=(
            "Nom du modèle LLM à utiliser "
            f"(par défaut: {DEFAULT_MODEL})."
        ),
    )
    parser.add_argument(
        "--api-key",
        type=str,
        default=None,
        help="Clé API pour le LLM. Si omise, utilise la variable LLM_API_KEY.",
    )
    parser.add_argument(
        "--api-base",
        type=str,
        default=os.getenv("LLM_API_BASE", "https://api.mistral.ai/v1"),
        help=(
            "URL de base de l'API LLM (OpenAI-compatible). "
            "Peut aussi être définie via LLM_API_BASE."
        ),
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="N'écrit pas le fichier, affiche le XML généré sur stdout.",
    )
    return parser.parse_args(argv)


def main(argv: Optional[List[str]] = None) -> int:
    args = parse_args(argv)

    # Récupération du prompt
    if args.prompt:
        natural_prompt = args.prompt
    else:
        natural_prompt = sys.stdin.read().strip()
    if not natural_prompt:
        print(
            "Erreur : aucun prompt fourni "
            "(option --prompt ou stdin vide).",
            file=sys.stderr,
        )
        return 1

    # Récupération des paramètres LLM
    api_key = args.api_key or os.getenv("LLM_API_KEY")
    if not api_key:
        print(
            "Erreur : aucune clé API LLM fournie.\n"
            "Utilisez --api-key ou définissez la variable "
            "d'environnement LLM_API_KEY.",
            file=sys.stderr,
        )
        return 1

    try:
        steps = call_llm_for_bt_steps(
            natural_language_prompt=natural_prompt,
            api_key=api_key,
            model=args.model,
            api_base=args.api_base,
        )
    except Exception as exc:
        print(f"Erreur lors de l'appel au LLM : {exc}", file=sys.stderr)
        return 1

    try:
        tree = build_bt_xml(steps)
    except Exception as exc:
        print(
            f"Erreur lors de la construction du Behavior Tree XML : {exc}",
            file=sys.stderr,
        )
        return 1

    if args.dry_run:
        # Affiche sur stdout pour inspection (sans xml_declaration)
        import io

        buffer = io.BytesIO()
        tree.write(buffer, encoding="utf-8", xml_declaration=False)
        print(buffer.getvalue().decode("utf-8"))
    else:
        output_path = Path(args.output).resolve()
        try:
            write_bt_xml(tree, output_path)
        except Exception as exc:
            print(
                f"Erreur lors de l'écriture du fichier XML : {exc}",
                file=sys.stderr,
            )
            return 1
        print(f"Behavior Tree généré dans : {output_path}")

    return 0


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
