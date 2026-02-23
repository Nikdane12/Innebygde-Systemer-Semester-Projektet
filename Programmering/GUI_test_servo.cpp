/*
 * 3D Robot Arm Visualizer — C++ Port
 * ====================================
 * Direct port of the Python/matplotlib arm visualizer.
 * Same FK (Ta×Tb), same IK solver, same drag-to-move behaviour.
 *
 * Dependencies:
 *   - SFML 2.6  (window, graphics, system)
 *   - Dear ImGui + imgui-sfml  (sliders, info panel)
 *   - C++17
 *
 * Build (Linux):
 *   sudo apt install libsfml-dev
 *   # Install imgui-sfml: https://github.com/SFML/imgui-sfml
 *   g++ -std=c++17 -O2 robot_arm.cpp \
 *       -lsfml-graphics -lsfml-window -lsfml-system \
 *       -lImGui-SFML -lGL \
 *       -o robot_arm
 *
 * Build (Windows / MSVC, vcpkg):
 *   vcpkg install sfml imgui[sfml-binding]
 *   cl /std:c++17 /O2 robot_arm.cpp /link sfml-graphics.lib sfml-window.lib \
 *      sfml-system.lib ImGui-SFML.lib opengl32.lib
 *
 * Build (macOS, Homebrew):
 *   brew install sfml
 *   # Build imgui-sfml from source
 *   g++ -std=c++17 -O2 robot_arm.cpp \
 *       -lsfml-graphics -lsfml-window -lsfml-system \
 *       -lImGui-SFML \
 *       -framework OpenGL \
 *       -o robot_arm
 *
 * Controls:
 *   Drag the red D marker with the mouse  →  IK mode (arm follows cursor)
 *   Sliders in the right panel            →  manual joint control
 *   Reset button                          →  return to home pose
 */

#include <SFML/Graphics.hpp>
#include <imgui.h>
#include <imgui-SFML.h>

#include <cmath>
#include <array>
#include <vector>
#include <optional>
#include <string>
#include <algorithm>
#include <sstream>
#include <iomanip>

// ─────────────────────────────────────────────────────────────────────────────
// Constants
// ─────────────────────────────────────────────────────────────────────────────
static constexpr float BOX_H = 0.35f;
static constexpr float L1    = 1.00f;
static constexpr float L2    = 0.80f;
static constexpr float L3    = 0.60f;
static constexpr float REACH = L1 + L2 + L3;

static constexpr float YAW_MIN = -90.f, YAW_MAX =  90.f;
static constexpr float SH_MIN  = -90.f, SH_MAX  =  90.f;
static constexpr float EL_MIN  =-135.f, EL_MAX  = 135.f;
static constexpr float WR_MIN  =-135.f, WR_MAX  = 135.f;

// Window layout
static constexpr int WIN_W      = 980;
static constexpr int WIN_H      = 650;
static constexpr int PANEL_W    = 280;      // ImGui right panel width
static constexpr int VIEWPORT_W = WIN_W - PANEL_W;

// ─────────────────────────────────────────────────────────────────────────────
// Math helpers
// ─────────────────────────────────────────────────────────────────────────────
static inline float deg2rad(float d) { return d * (float)M_PI / 180.f; }
static inline float rad2deg(float r) { return r * 180.f / (float)M_PI; }
static inline float clamp(float v, float lo, float hi) { return std::max(lo, std::min(hi, v)); }

static float wrap180(float deg) {
    while (deg >=  180.f) deg -= 360.f;
    while (deg <  -180.f) deg += 360.f;
    return deg;
}

// ─────────────────────────────────────────────────────────────────────────────
// 4×4 homogeneous transform — stored row-major in a flat array[16]
// T[row*4 + col]
// ─────────────────────────────────────────────────────────────────────────────
using Mat4 = std::array<float, 16>;

static Mat4 identity() {
    return {1,0,0,0,
            0,1,0,0,
            0,0,1,0,
            0,0,0,1};
}

static Mat4 mul44(const Mat4& A, const Mat4& B) {
    Mat4 C{};
    for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 4; ++c)
            for (int k = 0; k < 4; ++k)
                C[r*4+c] += A[r*4+k] * B[k*4+c];
    return C;
}

static Mat4 T_trans(float x, float y, float z) {
    auto T = identity();
    T[3] = x;  T[7] = y;  T[11] = z;
    return T;
}

// Standard Roty — rotates around Y axis
// Note: we pass -angle at call sites so +slider value lifts arm up
static Mat4 T_roty(float q) {
    float c = std::cos(q), s = std::sin(q);
    return { c, 0, s, 0,
             0, 1, 0, 0,
            -s, 0, c, 0,
             0, 0, 0, 1 };
}

static Mat4 T_rotz(float q) {
    float c = std::cos(q), s = std::sin(q);
    return { c,-s, 0, 0,
             s, c, 0, 0,
             0, 0, 1, 0,
             0, 0, 0, 1 };
}

// Extract translation (position) from a transform
struct Vec3 { float x, y, z; };
static Vec3 posFromT(const Mat4& T) { return { T[3], T[7], T[11] }; }

// ─────────────────────────────────────────────────────────────────────────────
// Forward kinematics
// Ta × Tb decomposition (studywolf method):
//   Ta = T_roty(-angle)    — joint rotation, negated so +angle lifts arm up
//   Tb = T_trans(L, 0, 0)  — static link offset
// ─────────────────────────────────────────────────────────────────────────────
struct FKResult { Vec3 O, A, B, C, D; };

static FKResult fk_chain(float yaw_deg, float sh_deg, float el_deg, float wr_deg) {
    float yaw = deg2rad(yaw_deg);
    float sh  = deg2rad(sh_deg);
    float el  = deg2rad(el_deg);
    float wr  = deg2rad(wr_deg);

    Mat4 T_O = identity();
    Mat4 T_A = mul44(mul44(T_O, T_rotz(yaw)), T_trans(0, 0, BOX_H));
    Mat4 T_B = mul44(T_A, mul44(T_roty(-sh), T_trans(L1, 0, 0)));
    Mat4 T_C = mul44(T_B, mul44(T_roty(-el), T_trans(L2, 0, 0)));
    Mat4 T_D = mul44(T_C, mul44(T_roty(-wr), T_trans(L3, 0, 0)));

    return { posFromT(T_O), posFromT(T_A), posFromT(T_B),
             posFromT(T_C), posFromT(T_D) };
}

// ─────────────────────────────────────────────────────────────────────────────
// IK: 3-link plane solver with continuity seeding
// ─────────────────────────────────────────────────────────────────────────────
struct IKAngles { float shoulder, elbow, wrist; };

static IKAngles ik_solve(float r_forward, float z_world,
                          std::optional<IKAngles> prev = std::nullopt)
{
    float px = std::max(0.f, r_forward);
    float pz = z_world - BOX_H;
    float base_tool = std::atan2(pz, px > 1e-9f ? px : 1e-9f);

    // Candidate tool angles
    std::vector<float> cands;
    cands.reserve(60 + 25 + (prev ? 21 : 0));
    for (int d = -90; d <= 90; d += 5)
        cands.push_back(base_tool + deg2rad((float)d));
    for (int d = -180; d <= 180; d += 15)
        cands.push_back(deg2rad((float)d));
    if (prev) {
        float pt = deg2rad(prev->shoulder + prev->elbow + prev->wrist);
        for (int d = -20; d <= 20; d += 2)
            cands.push_back(pt + deg2rad((float)d));
    }

    struct Best { float cost, sh, el, wr; };
    std::optional<Best> best;

    for (float tool : cands) {
        float cx = px - L3 * std::cos(tool);
        float cz = pz - L3 * std::sin(tool);
        float d2 = cx*cx + cz*cz;
        float d  = std::sqrt(d2);

        if (d > L1 + L2 || d < std::abs(L1 - L2)) continue;

        float cos_el = clamp((d2 - L1*L1 - L2*L2) / (2*L1*L2), -1.f, 1.f);
        float el0    = std::acos(cos_el);

        for (float er : { el0, -el0 }) {
            float sh  = std::atan2(cz, cx)
                      - std::atan2(L2*std::sin(er), L1 + L2*std::cos(er));
            float wr  = wrap180(rad2deg(tool - sh - er));
            float sh_d = rad2deg(sh);
            float el_d = rad2deg(er);

            if (sh_d < SH_MIN || sh_d > SH_MAX) continue;
            if (el_d < EL_MIN || el_d > EL_MAX) continue;
            if (wr   < WR_MIN || wr   > WR_MAX) continue;

            // In-plane FK error (validates the solution)
            float t2 = sh + er;
            float t3 = t2 + deg2rad(wr);
            float xf = L1*std::cos(sh) + L2*std::cos(t2) + L3*std::cos(t3);
            float zf = L1*std::sin(sh) + L2*std::sin(t2) + L3*std::sin(t3);
            float err = std::hypot(xf - px, zf - pz);

            float penalty = 0.001f * (std::abs(sh_d) + std::abs(el_d) + std::abs(wr));
            float cont    = 0.f;
            if (prev) {
                cont = 0.005f * (std::abs(sh_d - prev->shoulder)
                               + std::abs(el_d - prev->elbow)
                               + std::abs(wr   - prev->wrist));
            }
            float cost = err + penalty + cont;
            if (!best || cost < best->cost)
                best = { cost, sh_d, el_d, wr };
        }
    }

    if (!best) {
        // Fallback: clamp a direct solution
        float cx  = px - L3*std::cos(base_tool);
        float cz2 = pz - L3*std::sin(base_tool);
        float d2  = cx*cx + cz2*cz2;
        float d   = clamp(std::sqrt(d2), 1e-6f, L1+L2-1e-6f);
        float cos_el = clamp((d2-L1*L1-L2*L2)/(2*L1*L2), -1.f, 1.f);
        float er  = std::acos(cos_el);
        float sh  = std::atan2(cz2, cx) - std::atan2(L2*std::sin(er), L1+L2*std::cos(er));
        float wr  = wrap180(rad2deg(base_tool - sh - er));
        return { clamp(rad2deg(sh), SH_MIN, SH_MAX),
                 clamp(rad2deg(er), EL_MIN, EL_MAX),
                 clamp(wr,          WR_MIN, WR_MAX) };
    }

    return { best->sh, best->el, best->wr };
}

// ─────────────────────────────────────────────────────────────────────────────
// Oblique 3D → 2D projection
// Matches matplotlib's default 3D view (azimuth=-60°, elevation=30°)
// Returns pixel position within the viewport [0..VIEWPORT_W] x [0..WIN_H]
// ─────────────────────────────────────────────────────────────────────────────
struct Proj {
    float cx, cy;   // viewport centre in pixels
    float scale;    // world-units → pixels
    float az, el;   // azimuth and elevation (radians)

    Proj() {
        cx    = VIEWPORT_W * 0.5f;
        cy    = WIN_H      * 0.52f;
        scale = VIEWPORT_W * 0.17f;   // fits ±REACH in the viewport
        az    = deg2rad(-60.f);
        el    = deg2rad( 30.f);
    }

    // Project world point (wx,wy,wz) → screen pixel (sx,sy)
    sf::Vector2f project(float wx, float wy, float wz) const {
        // Rotate by azimuth around Z then tilt by elevation
        float ca = std::cos(az), sa = std::sin(az);
        float ce = std::cos(el), se = std::sin(el);

        // Azimuth rotation (around Z)
        float rx = ca*wx - sa*wy;
        float ry = sa*wx + ca*wy;
        float rz = wz;

        // Elevation tilt (around new X)
        float ex =  rx;
        float ey =  ce*ry - se*rz;
        float ez =  se*ry + ce*rz;   // depth (unused here)
        (void)ez;

        // Orthographic screen mapping: x→right, y→up (flip y for screen)
        float sx = cx + scale * ex;
        float sy = cy - scale * ey;
        return { sx, sy };
    }

    // Compute world-units-per-pixel for r and z at a given point,
    // by projecting tiny offsets and measuring screen displacement.
    // Used for drag calibration.
    void dragGains(float wx, float wy, float wz, float yaw_deg,
                   float& gr, float& gz) const
    {
        const float eps = 0.05f;
        float yr = deg2rad(yaw_deg);
        float rx_hat = std::cos(yr), ry_hat = std::sin(yr);

        auto p0 = project(wx, wy, wz);
        auto p1 = project(wx + eps*rx_hat, wy + eps*ry_hat, wz);
        auto p2 = project(wx, wy, wz + eps);

        float horiz_r = std::abs(p1.x - p0.x);
        float vert_z  = std::abs(p2.y - p0.y);

        gr = clamp(eps / std::max(horiz_r, 0.5f), 0.003f, 0.06f);
        gz = clamp(eps / std::max(vert_z,  0.5f), 0.003f, 0.06f);
    }
};

// ─────────────────────────────────────────────────────────────────────────────
// Draw the arm into an sf::RenderTarget
// ─────────────────────────────────────────────────────────────────────────────
static void drawArm(sf::RenderTarget& rt, const Proj& proj,
                    const FKResult& fk,
                    float target_r, float target_z, float yaw_deg,
                    bool d_hovered)
{
    // All joint positions as screen points
    const Vec3 joints[5] = { fk.O, fk.A, fk.B, fk.C, fk.D };
    sf::Vector2f sp[5];
    for (int i = 0; i < 5; ++i)
        sp[i] = proj.project(joints[i].x, joints[i].y, joints[i].z);

    // ── Shadow (arm projected onto Z=0 plane) ────────────────────────────
    for (int i = 0; i < 4; ++i) {
        auto a = proj.project(joints[i].x, joints[i].y, 0);
        auto b = proj.project(joints[i+1].x, joints[i+1].y, 0);
        sf::Vertex line[2] = {
            { a, sf::Color(80, 80, 80, 80) },
            { b, sf::Color(80, 80, 80, 80) }
        };
        rt.draw(line, 2, sf::Lines);
    }

    // ── Arm segments ─────────────────────────────────────────────────────
    // Same colour scheme as the Python version
    const sf::Color segColors[4] = {
        sf::Color(33,  150, 243),   // #2196F3 blue
        sf::Color(76,  175,  80),   // #4CAF50 green
        sf::Color(255, 152,   0),   // #FF9800 orange
        sf::Color(233,  30,  99),   // #E91E63 pink
    };

    for (int i = 0; i < 4; ++i) {
        // Draw thick line as a thin rectangle for nicer look
        sf::Vector2f a = sp[i], b = sp[i+1];
        sf::Vector2f dir = b - a;
        float len = std::hypot(dir.x, dir.y);
        if (len < 0.001f) continue;
        sf::Vector2f norm = { -dir.y / len, dir.x / len };
        float hw = 2.5f;   // half-width in pixels

        sf::ConvexShape seg(4);
        seg.setPoint(0, a + norm*hw);
        seg.setPoint(1, a - norm*hw);
        seg.setPoint(2, b - norm*hw);
        seg.setPoint(3, b + norm*hw);
        seg.setFillColor(segColors[i]);
        rt.draw(seg);
    }

    // ── Joint dots ───────────────────────────────────────────────────────
    for (int i = 0; i < 5; ++i) {
        bool isD = (i == 4);
        float r  = isD ? (d_hovered ? 10.f : 7.f) : 4.f;
        sf::CircleShape dot(r);
        dot.setOrigin(r, r);
        dot.setPosition(sp[i]);

        if (isD) {
            dot.setFillColor(sf::Color::White);
            dot.setOutlineThickness(2.f);
            dot.setOutlineColor(segColors[3]);
        } else {
            dot.setFillColor(sf::Color(0, 220, 220));
            dot.setOutlineThickness(0.f);
        }
        rt.draw(dot);
    }

    // ── IK Target crosshair ───────────────────────────────────────────────
    float yr = deg2rad(yaw_deg);
    float tx = target_r * std::cos(yr);
    float ty = target_r * std::sin(yr);
    float tz = target_z;
    auto tp = proj.project(tx, ty, tz);

    const float span = 8.f;   // crosshair arm length in pixels
    sf::Color tc(220, 50, 50);
    sf::Vertex cross[4] = {
        { tp + sf::Vector2f(-span, 0), tc },
        { tp + sf::Vector2f( span, 0), tc },
        { tp + sf::Vector2f(0, -span), tc },
        { tp + sf::Vector2f(0,  span), tc },
    };
    rt.draw(cross, 2, sf::Lines);
    rt.draw(cross+2, 2, sf::Lines);

    // Small circle around target
    sf::CircleShape tc2(5.f);
    tc2.setOrigin(5.f, 5.f);
    tc2.setPosition(tp);
    tc2.setFillColor(sf::Color::Transparent);
    tc2.setOutlineColor(tc);
    tc2.setOutlineThickness(1.f);
    rt.draw(tc2);

    // ── Joint labels ──────────────────────────────────────────────────────
    // (Labels drawn by caller via sf::Text; skipped here to keep this fn pure)
}

// ─────────────────────────────────────────────────────────────────────────────
// Main
// ─────────────────────────────────────────────────────────────────────────────
int main() {
    // ── Window ──────────────────────────────────────────────────────────────
    sf::RenderWindow window(
        sf::VideoMode(WIN_W, WIN_H),
        "3D Arm Visualizer (Drag D + Wrist)",
        sf::Style::Titlebar | sf::Style::Close
    );
    window.setFramerateLimit(60);
    ImGui::SFML::Init(window);

    // ── Font (for joint labels) ──────────────────────────────────────────────
    sf::Font font;
    bool hasFont = font.loadFromFile("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf");
    if (!hasFont)
        hasFont = font.loadFromFile("C:/Windows/Fonts/arial.ttf");   // Windows fallback

    // ── State ────────────────────────────────────────────────────────────────
    float yaw      = 0.f;
    float shoulder = 0.f;
    float elbow    = 0.f;
    float wrist    = 0.f;

    auto fk = fk_chain(yaw, shoulder, elbow, wrist);
    float target_r = std::hypot(fk.D.x, fk.D.y);
    float target_z = fk.D.z;

    bool   dragging   = false;
    float  drag_gr    = 0.015f;   // world-r per horizontal pixel
    float  drag_gz    = 0.015f;   // world-z per vertical pixel
    sf::Vector2i last_mouse;

    Proj proj;

    // Helper: get D's screen position
    auto getDScreen = [&]() -> sf::Vector2f {
        auto f = fk_chain(yaw, shoulder, elbow, wrist);
        return proj.project(f.D.x, f.D.y, f.D.z);
    };

    // Helper: sync target from current FK (used after manual slider move)
    auto syncTarget = [&]() {
        auto f = fk_chain(yaw, shoulder, elbow, wrist);
        target_r = std::hypot(f.D.x, f.D.y);
        target_z = f.D.z;
    };

    sf::Clock deltaClock;

    // ── Main loop ────────────────────────────────────────────────────────────
    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            ImGui::SFML::ProcessEvent(window, event);

            if (event.type == sf::Event::Closed)
                window.close();

            // ── Mouse press: start drag if clicking near D ─────────────
            if (event.type == sf::Event::MouseButtonPressed
                && event.mouseButton.button == sf::Mouse::Left
                && !ImGui::GetIO().WantCaptureMouse)
            {
                sf::Vector2f ms((float)event.mouseButton.x,
                                (float)event.mouseButton.y);
                sf::Vector2f ds = getDScreen();
                float dist = std::hypot(ms.x - ds.x, ms.y - ds.y);
                if (dist < 20.f) {
                    dragging   = true;
                    last_mouse = { event.mouseButton.x, event.mouseButton.y };
                    // Compute projection-accurate gains at drag start
                    auto f = fk_chain(yaw, shoulder, elbow, wrist);
                    proj.dragGains(f.D.x, f.D.y, f.D.z, yaw, drag_gr, drag_gz);
                }
            }

            if (event.type == sf::Event::MouseButtonReleased
                && event.mouseButton.button == sf::Mouse::Left)
            {
                dragging = false;
            }

            // ── Mouse move: update IK target ───────────────────────────
            if (event.type == sf::Event::MouseMoved && dragging
                && !ImGui::GetIO().WantCaptureMouse)
            {
                int mx = event.mouseMove.x;
                int my = event.mouseMove.y;
                float dx = (float)(mx - last_mouse.x);
                float dy = (float)(my - last_mouse.y);
                last_mouse = { mx, my };

                target_r = clamp(target_r + dx * drag_gr, 0.f, REACH);
                target_z = clamp(target_z - dy * drag_gz, 0.f, BOX_H + REACH);

                IKAngles prev { shoulder, elbow, wrist };
                auto sol = ik_solve(target_r, target_z, prev);
                shoulder = sol.shoulder;
                elbow    = sol.elbow;
                wrist    = sol.wrist;
            }
        }

        // ── ImGui UI (right panel) ────────────────────────────────────────
        ImGui::SFML::Update(window, deltaClock.restart());

        ImGui::SetNextWindowPos(ImVec2((float)VIEWPORT_W, 0), ImGuiCond_Always);
        ImGui::SetNextWindowSize(ImVec2((float)PANEL_W, (float)WIN_H), ImGuiCond_Always);
        ImGui::SetNextWindowBgAlpha(0.95f);
        ImGui::Begin("Controls", nullptr,
                     ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
                     ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoTitleBar);

        ImGui::TextUnformatted("3D Arm Visualizer");
        ImGui::Separator();

        // Sliders — same labels and ranges as Python version
        bool sliderMoved = false;

        ImGui::Text("Base yaw (-90..+90)");
        if (ImGui::SliderFloat("##yaw", &yaw, YAW_MIN, YAW_MAX, "%.1f deg"))
            sliderMoved = true;

        ImGui::Spacing();
        ImGui::Text("Shoulder pitch (actual)");
        if (ImGui::SliderFloat("##sh", &shoulder, SH_MIN, SH_MAX, "%.1f deg"))
            sliderMoved = true;

        ImGui::Spacing();
        ImGui::Text("Elbow pitch (relative)");
        if (ImGui::SliderFloat("##el", &elbow, EL_MIN, EL_MAX, "%.1f deg"))
            sliderMoved = true;

        ImGui::Spacing();
        ImGui::Text("Wrist pitch (relative)");
        if (ImGui::SliderFloat("##wr", &wrist, WR_MIN, WR_MAX, "%.1f deg"))
            sliderMoved = true;

        if (sliderMoved && !dragging)
            syncTarget();

        ImGui::Separator();

        // Info readout
        auto fkNow = fk_chain(yaw, shoulder, elbow, wrist);
        Vec3 D  = fkNow.D;
        float reach_pct = std::sqrt(D.x*D.x + D.y*D.y + D.z*D.z) / REACH * 100.f;
        const char* warn = reach_pct > 93.f ? "  ! near limit" : "";

        ImGui::TextUnformatted("End-effector (world):");
        ImGui::Text("  X = %+.3f m", D.x);
        ImGui::Text("  Y = %+.3f m", D.y);
        ImGui::Text("  Z = %+.3f m", D.z);
        ImGui::Text("  Reach: %.1f%%%s", reach_pct, warn);

        ImGui::Spacing();
        ImGui::TextUnformatted("IK target:");
        ImGui::Text("  r = %.3f m", target_r);
        ImGui::Text("  z = %.3f m", target_z);

        ImGui::Spacing();
        ImGui::TextUnformatted("Drag red D marker to move arm");
        ImGui::TextUnformatted("+shoulder = arm up");

        ImGui::Separator();

        if (ImGui::Button("Reset", ImVec2(-1.f, 0.f))) {
            yaw = shoulder = elbow = wrist = 0.f;
            syncTarget();
        }

        ImGui::End();

        // ── Render ────────────────────────────────────────────────────────
        window.clear(sf::Color(30, 30, 30));

        // Viewport background
        sf::RectangleShape vp(sf::Vector2f((float)VIEWPORT_W, (float)WIN_H));
        vp.setFillColor(sf::Color(20, 20, 25));
        window.draw(vp);

        // Hover check for D
        sf::Vector2i mp = sf::Mouse::getPosition(window);
        sf::Vector2f dSc = getDScreen();
        bool dHovered = !dragging &&
                        std::hypot((float)mp.x - dSc.x, (float)mp.y - dSc.y) < 20.f;

        // Arm
        auto fkDraw = fk_chain(yaw, shoulder, elbow, wrist);
        drawArm(window, proj, fkDraw, target_r, target_z, yaw, dHovered || dragging);

        // Joint labels
        if (hasFont) {
            const Vec3 jpts[5] = {
                fkDraw.O, fkDraw.A, fkDraw.B, fkDraw.C, fkDraw.D
            };
            const char* jlabels[5] = {"O","A","B","C","D"};
            for (int i = 0; i < 5; ++i) {
                auto sp = proj.project(jpts[i].x, jpts[i].y, jpts[i].z);
                sf::Text lbl(jlabels[i], font, 13);
                lbl.setFillColor(sf::Color(180,180,180));
                lbl.setPosition(sp.x + 6, sp.y - 16);
                window.draw(lbl);
            }
        }

        // Panel separator line
        sf::RectangleShape sep(sf::Vector2f(1.f, (float)WIN_H));
        sep.setPosition((float)VIEWPORT_W, 0.f);
        sep.setFillColor(sf::Color(60, 60, 60));
        window.draw(sep);

        ImGui::SFML::Render(window);
        window.display();
    }

    ImGui::SFML::Shutdown();
    return 0;
}