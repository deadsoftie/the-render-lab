#pragma once

struct ImVec4;

enum class StatusColor
{
    Muted,
    Info,
    Warning,
    Danger,
};

class EditorTheme
{
   public:
    EditorTheme() = delete;

    static void Apply();
    static void ScaleForDpi(float scale);
    static ImVec4 Color(StatusColor status, float alpha = 1.0f);
};
