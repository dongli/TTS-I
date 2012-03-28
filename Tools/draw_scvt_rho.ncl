load "$NCARG_ROOT/lib/ncarg/nclscripts/csm/gsn_code.ncl"
load "$NCARG_ROOT/lib/ncarg/nclscripts/csm/gsn_csm.ncl"
load "$NCARG_ROOT/lib/ncarg/nclscripts/csm/contributed.ncl"

load "$GEODIAG_UTILS/geodiag_plot_utils.ncl"

begin

    if (.not. isdefined("file_name")) then
        notice("Input the density function file:")
        file_name = get_answer()
    end if

    f = addfile(file_name, "r")

    wks = gsn_open_wks("pdf", systemfunc("basename -s .nc "+file_name))

    gsn_define_colormap(wks, "WhiteYellowOrangeRed")

    res = True
    res@gsnMaximize = True
    res@cnFillMode = "RasterFill"
    res@gsnAddCyclic = False
    ;res@mpOutlineOn = False
    setup_contour(res)
    set_contour_range(res, 0.1, 1.0, 0.1, "%3.1f")

    plot = gsn_csm_contour_map_ce(wks, f->rho, res)

    draw(plot)
    frame(wks)

end
