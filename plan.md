# UI Layout Fix Plan for gs/src/main.cpp

The current UI layout has an issue where the new control panel pushes the RSSI and other status indicators down. This plan corrects the layout by restructuring the ImGui windows to have a main left panel for the video and status indicators, and a right panel for the controls.

## Proposed Changes

1.  **Create a `LeftPanel` container:** This new `ImGui::BeginChild` will act as the main container for the left side of the screen.
2.  **Adjust `VideoPane` size:** The `VideoPane` will be placed inside the `LeftPanel` and its height will be adjusted to leave space for the status bar at the bottom.
3.  **Move status indicators:** The RSSI and other status indicators will be moved inside the `LeftPanel`, appearing below the `VideoPane`.

## Code Diff for gs/src/main.cpp

```diff
<<<<<<< SEARCH
:start_line:1194
-------
            // Child window for Video + OSD status
            ImGui::BeginChild("VideoPane", ImVec2(display_size.x - control_panel_width, 0), false, ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
            
            g_osd.draw();
            ImGui::EndChild();

            ImGui::SameLine();

            // Right panel container
            ImGui::BeginChild("RightPanel", ImVec2(control_panel_width, 0), false);

            // Child window for Controls
            ImGui::BeginChild("ControlPane", ImVec2(control_panel_width, 300), true);
            {
                // State for the toggle button
                static bool gpio_pin_state = false;

                ImGui::Spacing();
                ImGui::Text("Controles");
                ImGui::Separator();
                ImGui::Spacing();

                // Change button text and color based on state
                const char* button_text = gpio_pin_state ? "GPIO PIN: ON" : "GPIO PIN: OFF";
                ImVec4 button_color = gpio_pin_state ? ImVec4(0.2f, 0.7f, 0.2f, 1.0f) : ImVec4(0.8f, 0.2f, 0.2f, 1.0f);
                ImVec4 hover_color = gpio_pin_state ? ImVec4(0.3f, 0.8f, 0.3f, 1.0f) : ImVec4(0.9f, 0.3f, 0.3f, 1.0f);
                ImVec4 active_color = gpio_pin_state ? ImVec4(0.1f, 0.6f, 0.1f, 1.0f) : ImVec4(0.7f, 0.1f, 0.1f, 1.0f);

                ImGui::PushStyleColor(ImGuiCol_Button, button_color);
                ImGui::PushStyleColor(ImGuiCol_ButtonHovered, hover_color);
                ImGui::PushStyleColor(ImGuiCol_ButtonActive, active_color);

                if (ImGui::Button(button_text, ImVec2(-1, 50))) // -1 width = fill available space
                {
                    gpio_pin_state = !gpio_pin_state; // Toggle the state
                    config.dataChannel.gpio_control_btn++; // Increment the counter to notify the ESP32
                }

                ImGui::PopStyleColor(3);
            }
            ImGui::EndChild();
            ImGui::EndChild(); // End RightPanel

            {
                //RC RSSI
=======
            // Left panel container
            ImGui::BeginChild("LeftPanel", ImVec2(display_size.x - control_panel_width, 0), false);

            // Child window for Video + OSD status
            ImGui::BeginChild("VideoPane", ImVec2(display_size.x - control_panel_width, display_size.y - 40), false, ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
            g_osd.draw();
            ImGui::EndChild();

            // Status bar
            {
                //RC RSSI
>>>>>>> REPLACE