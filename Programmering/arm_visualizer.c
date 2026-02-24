/*
 * 3D Robotic Arm Visualizer — C port of the Python/Tkinter version
 * Deps: gtk+-3.0, cairo (usually bundled with GTK)
 * Build: gcc arm_visualizer.c -o arm_visualizer $(pkg-config --cflags --libs gtk+-3.0) -lm
 */

#include <gtk/gtk.h>
#include <cairo.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

/* ─── Robot Geometry ──────────────────────────────────────────── */
#define BOX_H 0.35
#define L1    1.00
#define L2    0.80
#define L3    0.60

#define YAW_MIN -90.0
#define YAW_MAX  90.0
#define SH_MIN  -90.0
#define SH_MAX   90.0
#define EL_MIN -135.0
#define EL_MAX  135.0
#define WR_MIN -135.0
#define WR_MAX  135.0

/* ─── Math helpers ────────────────────────────────────────────── */
static double deg2rad(double d) { return d * M_PI / 180.0; }
static double rad2deg(double r) { return r * 180.0 / M_PI; }
static double clamp(double v, double lo, double hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}
static double wrap180(double deg) {
    while (deg >= 180.0)  deg -= 360.0;
    while (deg < -180.0)  deg += 360.0;
    return deg;
}

/* ─── 4×4 homogeneous transforms ─────────────────────────────── */
typedef double Mat4[4][4];

static void mat_identity(Mat4 m) {
    memset(m, 0, sizeof(Mat4));
    m[0][0] = m[1][1] = m[2][2] = m[3][3] = 1.0;
}

static void mat_mul(const Mat4 A, const Mat4 B, Mat4 C) {
    Mat4 tmp;
    memset(tmp, 0, sizeof(Mat4));
    for (int r = 0; r < 4; r++)
        for (int c = 0; c < 4; c++)
            for (int k = 0; k < 4; k++)
                tmp[r][c] += A[r][k] * B[k][c];
    memcpy(C, tmp, sizeof(Mat4));
}

static void mat_trans(double x, double y, double z, Mat4 m) {
    mat_identity(m);
    m[0][3] = x; m[1][3] = y; m[2][3] = z;
}

static void mat_roty(double rad, Mat4 m) {
    double c = cos(rad), s = sin(rad);
    mat_identity(m);
    m[0][0] =  c; m[0][2] =  s;
    m[2][0] = -s; m[2][2] =  c;
}

static void mat_rotz(double rad, Mat4 m) {
    double c = cos(rad), s = sin(rad);
    mat_identity(m);
    m[0][0] =  c; m[0][1] = -s;
    m[1][0] =  s; m[1][1] =  c;
}

typedef struct { double x, y, z; } Vec3;

static Vec3 point_from_mat(const Mat4 m) {
    return (Vec3){ m[0][3], m[1][3], m[2][3] };
}

/* ─── Forward Kinematics ─────────────────────────────────────── */
static void fk_chain(double yaw_deg, double sh_deg, double el_deg, double wr_deg,
                     Vec3 joints[5])
{
    double yaw = deg2rad(yaw_deg);
    double sh  = deg2rad(sh_deg);
    double el  = deg2rad(el_deg);
    double wr  = deg2rad(wr_deg);

    Mat4 T_O, T_yaw, T_OA, T_AB, T_BC, T_CD;
    Mat4 T_A, T_B, T_C, T_D;
    Mat4 tmp1, tmp2;

    mat_identity(T_O);
    mat_rotz(yaw, T_yaw);
    mat_trans(0, 0, BOX_H, T_OA);

    /* Negate sh/el/wr so +angle lifts arm upward */
    Mat4 R_sh, Tr1, R_el, Tr2, R_wr, Tr3;
    mat_roty(-sh, R_sh); mat_trans(L1, 0, 0, Tr1); mat_mul(R_sh, Tr1, T_AB);
    mat_roty(-el, R_el); mat_trans(L2, 0, 0, Tr2); mat_mul(R_el, Tr2, T_BC);
    mat_roty(-wr, R_wr); mat_trans(L3, 0, 0, Tr3); mat_mul(R_wr, Tr3, T_CD);

    /* T_A = T_O * T_yaw * T_OA */
    mat_mul(T_O, T_yaw, tmp1); mat_mul(tmp1, T_OA, T_A);
    mat_mul(T_A, T_AB, T_B);
    mat_mul(T_B, T_BC, T_C);
    mat_mul(T_C, T_CD, T_D);

    joints[0] = point_from_mat(T_O);
    joints[1] = point_from_mat(T_A);
    joints[2] = point_from_mat(T_B);
    joints[3] = point_from_mat(T_C);
    joints[4] = point_from_mat(T_D);
}

/* ─── 3-link IK in arm plane ─────────────────────────────────── */
typedef struct { double shoulder, elbow, wrist; } IKResult;

static IKResult ik_3link_plane(double r_forward, double z_world,
                                const IKResult *prev)
{
    if (r_forward < 0.0) r_forward = 0.0;
    double px = r_forward;
    double pz = z_world - BOX_H;

    double base_tool = atan2(pz, px > 1e-9 ? px : 1e-9);

    IKResult best = {0};
    double   best_cost = 1e18;
    int      found = 0;

    /* Build candidate tool angles */
    int n_cands = 0;
    double cands[800];

    for (int ddeg = -90; ddeg <= 90; ddeg += 5)
        cands[n_cands++] = base_tool + deg2rad((double)ddeg);
    for (int ddeg = -180; ddeg <= 180; ddeg += 15)
        cands[n_cands++] = deg2rad((double)ddeg);
    if (prev) {
        double prev_tool = deg2rad(prev->shoulder + prev->elbow + prev->wrist);
        for (int ddeg = -20; ddeg <= 20; ddeg += 2)
            cands[n_cands++] = prev_tool + deg2rad((double)ddeg);
    }

    for (int ci = 0; ci < n_cands; ci++) {
        double tool = cands[ci];
        double cx = px - L3 * cos(tool);
        double cz = pz - L3 * sin(tool);

        double d2 = cx*cx + cz*cz;
        double d  = sqrt(d2);
        double a = L1, b = L2;

        if (d > a + b || d < fabs(a - b)) continue;

        double cos_el = (d2 - a*a - b*b) / (2*a*b);
        cos_el = clamp(cos_el, -1.0, 1.0);
        double el0 = acos(cos_el);

        for (int branch = 0; branch < 2; branch++) {
            double elbow_rel = (branch == 0) ? el0 : -el0;
            double sh = atan2(cz, cx) - atan2(b*sin(elbow_rel), a + b*cos(elbow_rel));
            double wr_raw = tool - sh - elbow_rel;

            double sh_slider = rad2deg(sh);
            double el_deg    = rad2deg(elbow_rel);
            double wr_deg    = wrap180(rad2deg(wr_raw));

            if (sh_slider < SH_MIN || sh_slider > SH_MAX) continue;
            if (el_deg    < EL_MIN || el_deg    > EL_MAX) continue;
            if (wr_deg    < WR_MIN || wr_deg    > WR_MAX) continue;

            /* FK error */
            double th1 = sh;
            double th2 = sh + elbow_rel;
            double th3 = sh + elbow_rel + deg2rad(wr_deg);
            double x_fk = L1*cos(th1) + L2*cos(th2) + L3*cos(th3);
            double z_fk = L1*sin(th1) + L2*sin(th2) + L3*sin(th3);
            double err = hypot(x_fk - px, z_fk - pz);

            double penalty = 0.001*(fabs(sh_slider)+fabs(el_deg)+fabs(wr_deg));
            double cont = 0.0;
            if (prev)
                cont = 0.005*(fabs(sh_slider - prev->shoulder) +
                              fabs(el_deg    - prev->elbow)    +
                              fabs(wr_deg    - prev->wrist));

            double cost = err + penalty + cont;
            if (!found || cost < best_cost) {
                best_cost = cost;
                best.shoulder = sh_slider;
                best.elbow    = el_deg;
                best.wrist    = wr_deg;
                found = 1;
            }
        }
    }

    /* Fallback */
    if (!found) {
        double tool = base_tool;
        double cx = px - L3*cos(tool);
        double cz = pz - L3*sin(tool);
        double d2 = cx*cx + cz*cz;
        double d  = clamp(sqrt(d2), 1e-6, L1+L2-1e-6);
        double cos_el = clamp((d2-L1*L1-L2*L2)/(2*L1*L2), -1.0, 1.0);
        double elbow_rel = acos(cos_el);
        double sh = atan2(cz,cx) - atan2(L2*sin(elbow_rel), L1+L2*cos(elbow_rel));
        double wr = tool - sh - elbow_rel;
        best.shoulder = clamp(rad2deg(sh), SH_MIN, SH_MAX);
        best.elbow    = clamp(rad2deg(elbow_rel), EL_MIN, EL_MAX);
        best.wrist    = clamp(wrap180(rad2deg(wr)), WR_MIN, WR_MAX);
    }
    return best;
}

/* ─── App state ──────────────────────────────────────────────── */
static struct {
    GtkWidget *canvas;
    GtkWidget *lbl_info;
    GtkWidget *sl_yaw, *sl_sh, *sl_el, *sl_wr;
    double target_r, target_z;
    gboolean dragging;
    double last_mx, last_my;
    gboolean internal_update;

    /* 3D→2D projection params */
    double view_elev;  /* degrees */
    double view_azim;  /* degrees */
} App;

/* ─── Simple orthographic 3D→2D projection ───────────────────── */
static void project(double x3, double y3, double z3,
                    double cx, double cy, double scale,
                    double *sx, double *sy)
{
    double elev = deg2rad(App.view_elev);
    double azim = deg2rad(App.view_azim);

    /* Rotate around Z by azimuth, then around X by elevation */
    double xr =  x3 * cos(azim) + y3 * sin(azim);
    double yr = -x3 * sin(azim) + y3 * cos(azim);
    double zr =  z3;

    double xp = xr;
    double yp = yr * cos(elev) - zr * sin(elev);
    /* z-depth (unused for ortho) */

    *sx = cx + xp * scale;
    *sy = cy + yp * scale;  /* screen y increases downward */
}

/* ─── Repaint ────────────────────────────────────────────────── */
static gboolean on_draw(GtkWidget *w, cairo_t *cr, gpointer data)
{
    (void)w; (void)data;

    int W = gtk_widget_get_allocated_width(w);
    int H = gtk_widget_get_allocated_height(w);

    /* Dark background */
    cairo_set_source_rgb(cr, 0.12, 0.12, 0.18);
    cairo_paint(cr);

    double yaw = gtk_range_get_value(GTK_RANGE(App.sl_yaw));
    double shs = gtk_range_get_value(GTK_RANGE(App.sl_sh));
    double el  = gtk_range_get_value(GTK_RANGE(App.sl_el));
    double wr  = gtk_range_get_value(GTK_RANGE(App.sl_wr));

    Vec3 joints[5];
    fk_chain(yaw, shs, el, wr, joints);

    double reach = L1 + L2 + L3;
    double scale = (double)H * 0.35 / reach;
    double cx = W * 0.47;
    double cy = H * 0.52;

    /* Helper: project one point */
    #define PROJ(pt, sx, sy) project((pt).x,(pt).y,(pt).z,cx,cy,scale,&(sx),&(sy))

    /* ── Ground grid ── */
    cairo_set_source_rgba(cr, 0.3, 0.3, 0.4, 0.4);
    cairo_set_line_width(cr, 0.5);
    for (int i = -5; i <= 5; i++) {
        double v = i * 0.5;
        double ax_, ay_, bx_, by_;
        project(v, -2.5, 0, cx, cy, scale, &ax_, &ay_);
        project(v,  2.5, 0, cx, cy, scale, &bx_, &by_);
        cairo_move_to(cr, ax_, ay_); cairo_line_to(cr, bx_, by_);
        cairo_stroke(cr);
        project(-2.5, v, 0, cx, cy, scale, &ax_, &ay_);
        project( 2.5, v, 0, cx, cy, scale, &bx_, &by_);
        cairo_move_to(cr, ax_, ay_); cairo_line_to(cr, bx_, by_);
        cairo_stroke(cr);
    }

    /* ── Arm shadow on ground ── */
    cairo_set_source_rgba(cr, 0.5, 0.5, 0.5, 0.25);
    cairo_set_line_width(cr, 2);
    double s0x, s0y;
    PROJ(joints[0], s0x, s0y);
    cairo_move_to(cr, s0x, s0y);
    for (int i = 1; i < 5; i++) {
        Vec3 sh = { joints[i].x, joints[i].y, 0.0 };
        double sx_, sy_;
        project(sh.x, sh.y, 0, cx, cy, scale, &sx_, &sy_);
        cairo_line_to(cr, sx_, sy_);
    }
    cairo_stroke(cr);

    /* ── Arm segments ── */
    double seg_r[] = {0.13, 0.25, 1.00, 0.91};
    double seg_g[] = {0.59, 0.75, 0.60, 0.12};
    double seg_b[] = {0.95, 0.25, 0.00, 0.47};

    cairo_set_line_width(cr, 5);
    for (int i = 0; i < 4; i++) {
        double ax_, ay_, bx_, by_;
        PROJ(joints[i],   ax_, ay_);
        PROJ(joints[i+1], bx_, by_);
        cairo_set_source_rgb(cr, seg_r[i], seg_g[i], seg_b[i]);
        cairo_move_to(cr, ax_, ay_);
        cairo_line_to(cr, bx_, by_);
        cairo_stroke(cr);
    }

    /* ── Joint markers ── */
    const char *labels[] = {"O","A","B","C","D"};
    for (int i = 0; i < 5; i++) {
        double jx_, jy_;
        PROJ(joints[i], jx_, jy_);
        double r = (i == 4) ? 9.0 : 5.0;
        if (i == 4)
            cairo_set_source_rgb(cr, 1.0, 1.0, 1.0);
        else
            cairo_set_source_rgb(cr, 0.0, 1.0, 1.0);
        cairo_arc(cr, jx_, jy_, r, 0, 2*M_PI);
        cairo_fill(cr);
        cairo_set_source_rgb(cr, 0.8, 0.8, 0.8);
        cairo_move_to(cr, jx_+r+2, jy_-r-2);
        cairo_select_font_face(cr, "Sans", CAIRO_FONT_SLANT_NORMAL, CAIRO_FONT_WEIGHT_BOLD);
        cairo_set_font_size(cr, 11);
        cairo_show_text(cr, labels[i]);
    }

    /* ── Target marker ── */
    {
        double yaw_r = deg2rad(yaw);
        double tx = App.target_r * cos(yaw_r);
        double ty = App.target_r * sin(yaw_r);
        double tz = App.target_z;
        double tsx, tsy;
        project(tx, ty, tz, cx, cy, scale, &tsx, &tsy);

        cairo_set_source_rgba(cr, 1.0, 0.2, 0.2, 0.85);
        cairo_set_line_width(cr, 1.5);
        double span = 8.0;
        cairo_move_to(cr, tsx-span, tsy-span); cairo_line_to(cr, tsx+span, tsy+span); cairo_stroke(cr);
        cairo_move_to(cr, tsx+span, tsy-span); cairo_line_to(cr, tsx-span, tsy+span); cairo_stroke(cr);
        cairo_arc(cr, tsx, tsy, span, 0, 2*M_PI);
        cairo_stroke(cr);
    }

    /* ── Axis legend (bottom-left) ── */
    {
        double ox = 60, oy = H - 50;
        double alen = 30;
        struct { double dx, dy, dz; const char *lbl; double r,g,b; } axes[] = {
            {1,0,0,"X",1,0.3,0.3},{0,1,0,"Y",0.3,1,0.3},{0,0,1,"Z",0.3,0.3,1}
        };
        for (int a = 0; a < 3; a++) {
            double ex, ey;
            project(axes[a].dx, axes[a].dy, axes[a].dz, ox, oy, alen, &ex, &ey);
            cairo_set_source_rgb(cr, axes[a].r, axes[a].g, axes[a].b);
            cairo_set_line_width(cr, 2);
            cairo_move_to(cr, ox, oy); cairo_line_to(cr, ex, ey); cairo_stroke(cr);
            cairo_move_to(cr, ex+2, ey-2);
            cairo_set_font_size(cr, 10);
            cairo_show_text(cr, axes[a].lbl);
        }
    }

    /* ── Info text (top-left) ── */
    {
        Vec3 D = joints[4];
        double reach_pct = sqrt(D.x*D.x + D.y*D.y + D.z*D.z) / reach * 100.0;
        char info[512];
        snprintf(info, sizeof(info),
            "Yaw:      %+.1f°\n"
            "Shoulder: %+.1f°\n"
            "Elbow:    %+.1f°\n"
            "Wrist:    %+.1f°\n\n"
            "End-effector:\n"
            "  X = %+.3f m\n"
            "  Y = %+.3f m\n"
            "  Z = %+.3f m\n"
            "  Reach: %.1f%%%s\n\n"
            "IK target:\n"
            "  r = %.3f m\n"
            "  z = %.3f m",
            yaw, shs, el, wr,
            D.x, D.y, D.z,
            reach_pct, reach_pct > 93.0 ? "  ⚠" : "",
            App.target_r, App.target_z
        );

        cairo_set_source_rgba(cr, 0.0, 0.0, 0.0, 0.5);
        cairo_rectangle(cr, 8, 8, 195, 195);
        cairo_fill(cr);

        cairo_set_source_rgb(cr, 0.85, 0.9, 0.85);
        cairo_select_font_face(cr, "Monospace", CAIRO_FONT_SLANT_NORMAL, CAIRO_FONT_WEIGHT_NORMAL);
        cairo_set_font_size(cr, 11);

        double tx = 16, ty = 26;
        char *line = info;
        char *end;
        while ((end = strchr(line, '\n')) || *line) {
            char buf[128] = {0};
            if (end) {
                strncpy(buf, line, end-line);
                line = end+1;
            } else {
                strncpy(buf, line, sizeof(buf)-1);
                line += strlen(line);
            }
            cairo_move_to(cr, tx, ty);
            cairo_show_text(cr, buf);
            ty += 14;
            if (!*line) break;
        }
    }

    /* ── Help text ── */
    cairo_set_source_rgba(cr, 0.6, 0.6, 0.7, 0.8);
    cairo_select_font_face(cr, "Sans", CAIRO_FONT_SLANT_ITALIC, CAIRO_FONT_WEIGHT_NORMAL);
    cairo_set_font_size(cr, 10);
    cairo_move_to(cr, W - 260, H - 12);
    cairo_show_text(cr, "Left-drag: move arm tip  |  Right-drag: rotate view");

    #undef PROJ
    return FALSE;
}

/* ─── Update info label (sidebar) ───────────────────────────── */
static void update_info(void)
{
    double yaw = gtk_range_get_value(GTK_RANGE(App.sl_yaw));
    double shs = gtk_range_get_value(GTK_RANGE(App.sl_sh));
    double el  = gtk_range_get_value(GTK_RANGE(App.sl_el));
    double wr  = gtk_range_get_value(GTK_RANGE(App.sl_wr));
    Vec3 joints[5];
    fk_chain(yaw, shs, el, wr, joints);
    Vec3 D = joints[4];
    double reach = L1+L2+L3;
    double reach_pct = sqrt(D.x*D.x+D.y*D.y+D.z*D.z)/reach*100.0;
    char buf[512];
    snprintf(buf, sizeof(buf),
        "Yaw:       %+.1f°\n"
        "Shoulder:  %+.1f°\n"
        "Elbow:     %+.1f°\n"
        "Wrist:     %+.1f°\n\n"
        "End-effector:\n"
        "  X = %+.3f m\n"
        "  Y = %+.3f m\n"
        "  Z = %+.3f m\n"
        "  Reach: %.1f%%%s\n\n"
        "IK target:\n"
        "  r = %.3f m\n"
        "  z = %.3f m\n\n"
        "Left-drag canvas:\n  move arm tip\n"
        "Right-drag canvas:\n  rotate view",
        yaw, shs, el, wr,
        D.x, D.y, D.z,
        reach_pct, reach_pct > 93.0 ? " ⚠" : "",
        App.target_r, App.target_z);
    gtk_label_set_text(GTK_LABEL(App.lbl_info), buf);
}

/* ─── Slider callbacks ───────────────────────────────────────── */
static void on_slider_changed(GtkRange *r, gpointer data)
{
    (void)r; (void)data;
    if (App.internal_update) { gtk_widget_queue_draw(App.canvas); return; }

    /* Keep target in sync with FK */
    double yaw = gtk_range_get_value(GTK_RANGE(App.sl_yaw));
    double shs = gtk_range_get_value(GTK_RANGE(App.sl_sh));
    double el  = gtk_range_get_value(GTK_RANGE(App.sl_el));
    double wr  = gtk_range_get_value(GTK_RANGE(App.sl_wr));
    Vec3 joints[5];
    fk_chain(yaw, shs, el, wr, joints);
    Vec3 D = joints[4];
    App.target_r = hypot(D.x, D.y);
    App.target_z = D.z;

    update_info();
    gtk_widget_queue_draw(App.canvas);
}

/* ─── Mouse events (canvas) ─────────────────────────────────── */
static gboolean on_button_press(GtkWidget *w, GdkEventButton *ev, gpointer data)
{
    (void)w; (void)data;
    if (ev->button == 1 || ev->button == 3) {
        App.dragging = TRUE;
        App.last_mx = ev->x;
        App.last_my = ev->y;
        /* Distinguish drag type via button; store button in a flag */
    }
    return TRUE;
}
static gboolean on_button_release(GtkWidget *w, GdkEventButton *ev, gpointer data)
{
    (void)w; (void)ev; (void)data;
    App.dragging = FALSE;
    return TRUE;
}

static int last_button = 0;
static gboolean on_button_press2(GtkWidget *w, GdkEventButton *ev, gpointer data)
{
    (void)w; (void)data;
    last_button = ev->button;
    return on_button_press(w, ev, data);
}

static gboolean on_motion(GtkWidget *w, GdkEventMotion *ev, gpointer data)
{
    (void)w; (void)data;
    if (!App.dragging) return TRUE;

    double dx = ev->x - App.last_mx;
    double dy = ev->y - App.last_my;
    App.last_mx = ev->x;
    App.last_my = ev->y;

    if (last_button == 3) {
        /* Right drag: rotate view */
        App.view_azim += dx * 0.5;
        App.view_elev  = clamp(App.view_elev - dy * 0.3, -89, 89);
        gtk_widget_queue_draw(App.canvas);
        return TRUE;
    }

    /* Left drag: move arm tip */
    int cw = gtk_widget_get_allocated_width(App.canvas);
    double reach = L1+L2+L3;
    double gain = (2.0 * reach) / (double)(cw > 100 ? cw : 100) * 0.85;

    App.target_r = fmax(0.0, App.target_r + dx * gain);
    App.target_z = clamp(App.target_z - dy * gain * 0.7,
                         0.0, BOX_H + reach);

    IKResult prev = {
        gtk_range_get_value(GTK_RANGE(App.sl_sh)),
        gtk_range_get_value(GTK_RANGE(App.sl_el)),
        gtk_range_get_value(GTK_RANGE(App.sl_wr))
    };
    IKResult ik = ik_3link_plane(App.target_r, App.target_z, &prev);

    App.internal_update = TRUE;
    gtk_range_set_value(GTK_RANGE(App.sl_sh), ik.shoulder);
    gtk_range_set_value(GTK_RANGE(App.sl_el), ik.elbow);
    gtk_range_set_value(GTK_RANGE(App.sl_wr), ik.wrist);
    App.internal_update = FALSE;

    update_info();
    gtk_widget_queue_draw(App.canvas);
    return TRUE;
}

/* ─── Reset button ───────────────────────────────────────────── */
static void on_reset(GtkButton *btn, gpointer data)
{
    (void)btn; (void)data;
    App.internal_update = TRUE;
    gtk_range_set_value(GTK_RANGE(App.sl_yaw), 0.0);
    gtk_range_set_value(GTK_RANGE(App.sl_sh),  0.0);
    gtk_range_set_value(GTK_RANGE(App.sl_el),  0.0);
    gtk_range_set_value(GTK_RANGE(App.sl_wr),  0.0);
    App.internal_update = FALSE;

    Vec3 joints[5];
    fk_chain(0,0,0,0,joints);
    Vec3 D = joints[4];
    App.target_r = hypot(D.x, D.y);
    App.target_z = D.z;

    update_info();
    gtk_widget_queue_draw(App.canvas);
}

/* ─── main ───────────────────────────────────────────────────── */
int main(int argc, char **argv)
{
    gtk_init(&argc, &argv);

    /* Initial state */
    App.view_elev = 25.0;
    App.view_azim = 30.0;
    App.dragging = FALSE;
    App.internal_update = FALSE;

    /* Window */
    GtkWidget *win = gtk_window_new(GTK_WINDOW_TOPLEVEL);
    gtk_window_set_title(GTK_WINDOW(win), "3D Arm Visualizer (C/GTK)");
    gtk_window_set_default_size(GTK_WINDOW(win), 1020, 660);
    g_signal_connect(win, "destroy", G_CALLBACK(gtk_main_quit), NULL);

    GtkWidget *hbox = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 0);
    gtk_container_add(GTK_CONTAINER(win), hbox);

    /* ── Drawing canvas ── */
    App.canvas = gtk_drawing_area_new();
    gtk_widget_set_size_request(App.canvas, 700, 600);
    gtk_widget_set_hexpand(App.canvas, TRUE);
    gtk_widget_set_vexpand(App.canvas, TRUE);
    gtk_box_pack_start(GTK_BOX(hbox), App.canvas, TRUE, TRUE, 0);

    g_signal_connect(App.canvas, "draw", G_CALLBACK(on_draw), NULL);

    gtk_widget_add_events(App.canvas,
        GDK_BUTTON_PRESS_MASK | GDK_BUTTON_RELEASE_MASK | GDK_POINTER_MOTION_MASK);
    g_signal_connect(App.canvas, "button-press-event",   G_CALLBACK(on_button_press2), NULL);
    g_signal_connect(App.canvas, "button-release-event", G_CALLBACK(on_button_release), NULL);
    g_signal_connect(App.canvas, "motion-notify-event",  G_CALLBACK(on_motion), NULL);

    /* ── Right-side controls ── */
    GtkWidget *vbox = gtk_box_new(GTK_ORIENTATION_VERTICAL, 8);
    gtk_widget_set_margin_start(vbox, 12);
    gtk_widget_set_margin_end(vbox, 12);
    gtk_widget_set_margin_top(vbox, 12);
    gtk_widget_set_margin_bottom(vbox, 12);
    gtk_box_pack_start(GTK_BOX(hbox), vbox, FALSE, FALSE, 0);

    /* Info label */
    App.lbl_info = gtk_label_new("");
    gtk_label_set_justify(GTK_LABEL(App.lbl_info), GTK_JUSTIFY_LEFT);
    PangoFontDescription *fd = pango_font_description_from_string("Monospace 9");
    gtk_widget_override_font(App.lbl_info, fd);
    pango_font_description_free(fd);
    gtk_box_pack_start(GTK_BOX(vbox), App.lbl_info, FALSE, FALSE, 0);

    /* Sliders helper */
    #define MAKE_SLIDER(name, label, lo, hi, val) \
        gtk_box_pack_start(GTK_BOX(vbox), gtk_label_new(label), FALSE, FALSE, 0); \
        App.name = gtk_scale_new_with_range(GTK_ORIENTATION_HORIZONTAL, lo, hi, 0.5); \
        gtk_range_set_value(GTK_RANGE(App.name), val); \
        gtk_widget_set_size_request(App.name, 260, -1); \
        gtk_scale_set_value_pos(GTK_SCALE(App.name), GTK_POS_RIGHT); \
        gtk_box_pack_start(GTK_BOX(vbox), App.name, FALSE, FALSE, 0); \
        g_signal_connect(App.name, "value-changed", G_CALLBACK(on_slider_changed), NULL);

    MAKE_SLIDER(sl_yaw, "Base Yaw (−90 … +90°)",          YAW_MIN, YAW_MAX, 0.0)
    MAKE_SLIDER(sl_sh,  "Shoulder Pitch (+up / −down)",    SH_MIN,  SH_MAX,  0.0)
    MAKE_SLIDER(sl_el,  "Elbow Pitch (relative)",          EL_MIN,  EL_MAX,  0.0)
    MAKE_SLIDER(sl_wr,  "Wrist Pitch (relative)",          WR_MIN,  WR_MAX,  0.0)

    #undef MAKE_SLIDER

    /* Reset button */
    GtkWidget *btn = gtk_button_new_with_label("Reset");
    gtk_box_pack_start(GTK_BOX(vbox), btn, FALSE, FALSE, 8);
    g_signal_connect(btn, "clicked", G_CALLBACK(on_reset), NULL);

    /* Init target from FK */
    Vec3 joints[5];
    fk_chain(0,0,0,0,joints);
    Vec3 D = joints[4];
    App.target_r = hypot(D.x, D.y);
    App.target_z = D.z;

    update_info();
    gtk_widget_show_all(win);
    gtk_main();
    return 0;
}