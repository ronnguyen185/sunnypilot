"""Onroad button to engage COMMA_GATE / SOC2 (sets GateForceEngage one-shot)."""

from __future__ import annotations

import pyray as rl

from openpilot.common.params import Params
from openpilot.selfdrive.gate.gate_params import gate_soc2_enabled, write_gate_bench
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.widgets import Widget


def _gate_engage_ui_enabled() -> bool:
  """Show engage control whenever this device is configured as COMMA_GATE SOC2."""
  if gate_soc2_enabled():
    return True
  bundle = Params().get("CarPlatformBundle") or {}
  return isinstance(bundle, dict) and bundle.get("platform") == "COMMA_GATE"


class GateEngageButton(Widget):
  """Tap to engage openpilot when brand=gate (no cruise/LKAS buttons)."""

  def __init__(self, button_size: int):
    super().__init__()
    self._font = gui_app.font(FontWeight.BOLD)
    self._rect = rl.Rectangle(0, 0, button_size, button_size)
    self._engageable = False
    self._enabled = False
    self._label = "ENGAGE"
    self._is_visible = False

  def set_rect(self, rect: rl.Rectangle) -> None:
    self._rect.x, self._rect.y = rect.x, rect.y
    self._rect.width = rect.width
    self._rect.height = rect.height

  def _update_state(self) -> None:
    self._is_visible = _gate_engage_ui_enabled()
    if not self._is_visible:
      return

    ss = ui_state.sm["selfdriveState"]
    self._enabled = bool(ss.enabled)
    # Show as tappable whenever not already enabled; grey if not engageable yet.
    self._engageable = bool(ss.engageable) or self._enabled
    self._label = "ON" if self._enabled else "ENGAGE"

  def handle_mouse_event(self) -> bool:
    if not self.is_visible:
      return False
    if not rl.check_collision_point_rec(rl.get_mouse_position(), self._rect):
      return False
    if rl.is_mouse_button_released(rl.MouseButton.MOUSE_BUTTON_LEFT):
      if not self._enabled:
        # Always pulse engage request; car_specific ignores if not engageable / brake held.
        write_gate_bench("GateForceEngage", "1")
    return True

  def _render(self, rect: rl.Rectangle) -> None:
    center_x = int(self._rect.x + self._rect.width / 2)
    center_y = int(self._rect.y + self._rect.height / 2)
    mouse_over = rl.check_collision_point_rec(rl.get_mouse_position(), self._rect)
    mouse_down = rl.is_mouse_button_down(rl.MouseButton.MOUSE_BUTTON_LEFT) and self.is_pressed

    if self._enabled:
      bg = rl.Color(23, 134, 68, 230)
      fg = rl.Color(255, 255, 255, 255)
      ring = rl.Color(128, 216, 166, 255)
    elif self._engageable:
      bg = rl.Color(20, 80, 160, 230)
      fg = rl.Color(255, 255, 255, 200 if (mouse_down and mouse_over) else 255)
      ring = rl.Color(100, 180, 255, 255)
    else:
      bg = rl.Color(40, 40, 40, 200)
      fg = rl.Color(200, 200, 200, 220)
      ring = rl.Color(120, 120, 120, 200)

    rl.draw_circle(center_x, center_y, self._rect.width / 2, bg)
    rl.draw_circle_lines(center_x, center_y, self._rect.width / 2 - 2, ring)
    rl.draw_circle_lines(center_x, center_y, self._rect.width / 2, rl.Color(255, 255, 255, 90))

    font_size = 40 if self._label == "ON" else 30
    text_size = measure_text_cached(self._font, self._label, font_size)
    rl.draw_text_ex(
      self._font,
      self._label,
      rl.Vector2(center_x - text_size.x / 2, center_y - text_size.y / 2),
      font_size,
      0,
      fg,
    )
