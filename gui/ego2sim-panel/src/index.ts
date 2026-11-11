import { ExtensionContext } from "@foxglove/extension";

import { initEgo2SimPanel } from "./Ego2SimPanel";

export function activate(extensionContext: ExtensionContext): void {
  extensionContext.registerPanel({ name: "Ego2Sim Control", initPanel: initEgo2SimPanel });
}
