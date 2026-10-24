#include "imgui.h"
#include <cstdio>
#include <cstdarg>
namespace ImGui {
    bool Begin(const char*, bool*) { return true; }
    void End() {}
    void Text(const char*, ...) {}
    void TextColored(const ImVec4&, const char*, ...) {}
    void SameLine(float, float) {}
    void Separator() {}
    void Spacing() {}
    void Columns(int, const char*, bool) {}
    void NextColumn() {}
    bool SliderFloat(const char*, float*, float, float, const char*) { return false; }
    bool SliderScalar(const char*, ImGuiDataType, void*, const void*, const void*) { return false; }
    bool Combo(const char*, int*, const char* const[], int, int) { return false; }
    void PlotLines(const char*, const float*, int, int, const char*, float, float, ImVec2) {}
    void PlotHistogram(const char*, const float*, int, int, const char*, float, float, ImVec2) {}
    float GetWindowWidth() { return 800.0f; }
    unsigned int PushID(const char*) { return 0; }
    void PopID() {}
}
