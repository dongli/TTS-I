load "$NCARG_ROOT/lib/ncarg/nclscripts/csm/gsn_code.ncl"
load "$NCARG_ROOT/lib/ncarg/nclscripts/csm/gsn_csm.ncl"
load "$NCARG_ROOT/lib/ncarg/nclscripts/csm/contributed.ncl"

load "/Users/dongli/Works/TTS/Tools/tts_plot_utils.ncl"

begin

    f = addfile("debug_gamil.suv.nc", "r")

    plot_type = "vector"

    wks = gsn_open_wks("newpdf", "velocity_field")

    projection = "SH"
    projection@min_lat = 80.0
    projection@max_lat = -80.0

    res_map = setup_map(projection)

    if (plot_type .eq. "vector") then
        do k = 0, 24;0, dimsizes(f->time)-1
            system("echo plotting time "+f->time(k))
            map = gsn_csm_map(wks, res_map)
            plot = show_velocity_vector(wks, f->u(k,:,:), f->v(k,:,:))
            ;plot = show_velocity_speed(wks, f->u(k,:,:), f->v(k,:,:))
            ;plot = show_velocity_contour(wks, f->u(k,:,:))
            overlay(map, plot)
            draw(map)
            frame(wks)
        end do
    end if

    if (plot_type .eq. "contour_u") then
        gsn_define_colormap(wks, "WhiteBlueGreenYellowRed")
        res                         = True
        res@gsnMaximize             = True
        res@cnFillOn                = True
        res@cnLinesOn               = False
        res@gsnSpreadColors         = True
        res@mpProjection            = "stereographic"
        res@gsnPolar                = "NH"
        res@mpMinLatF               = 80.0

        do k = 0, dimsizes(f->time)-1
            plot = gsn_csm_contour_map(wks, f->u(k,:,:), res)
        end do
    end if
end
