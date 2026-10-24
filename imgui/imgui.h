// imgui/imgui.h : Fully exposed GUI types matching main layout pipelines
#pragma once
#include <cstddef>
#include <cstdint>

// Move layout vectors into global visibility scope to resolve identifier crashes
struct ImVec2 { 
    float x, y; 
    ImVec2() : x(0.0f), y(0.0f) {} 
    ImVec2(float _x, float _y) : x(_x), y(_y) {} 
};

struct ImVec4 { 
    float x, y, z, w; 
    ImVec4() : x(0.0f), y(0.0f), z(0.0f), w(0.0f) {} 
    ImVec4(float _x, float _y, float _z, float _w) : x(_x), y(_y), z(_z), w(_w) {} 
};

// Global data types tracking
typedef int ImGuiDataType;
enum ImGuiDataType_ { 
    ImGuiDataType_U8 = 0 
};

namespace ImGui {
    bool Begin(const char* name, bool* p_open = nullptr);
    void End();
    void Text(const char* fmt, ...);
    void TextColored(const ImVec4& col, const char* fmt, ...);
    void SameLine(float offset_from_start_x = 0.0f, float spacing = -1.0f);
    void Separator();
    void Spacing();
    void Columns(int count = 1, const char* id = nullptr, bool border = true);
    void NextColumn();
    bool SliderFloat(const char* label, float* v, float v_min, float v_max, const char* format = "%.3f");
    bool SliderScalar(const char* label, ImGuiDataType data_type, void* p_data, const void* p_min, const void* p_max);
    bool Combo(const char* label, int* current_item, const char* const items[], int items_count, int popup_max_height_in_items = -1);
    void PlotLines(const char* label, const float* values, int values_count, int values_offset = 0, const char* overlay_text = nullptr, float scale_min = 3.4e38f, float scale_max = 3.4e38f, ImVec2 graph_size = ImVec2(0,0));
    void PlotHistogram(const char* label, const float* values, int values_count, int values_offset = 0, const char* overlay_text = nullptr, float scale_min = 3.4e38f, float scale_max = 3.4e38f, ImVec2 graph_size = ImVec2(0,0));
    float GetWindowWidth();
    unsigned int PushID(const char* str_id);
    void PopID();
}
