load "$NCARG_ROOT/lib/ncarg/nclscripts/csm/gsn_code.ncl"
load "$NCARG_ROOT/lib/ncarg/nclscripts/csm/gsn_csm.ncl"
load "$NCARG_ROOT/lib/ncarg/nclscripts/csm/contributed.ncl"

load "/Users/dongli/Works/TTS/Tools/tts_plot_utils.ncl"

begin

    file_name_pattern = ask("Input file name pattern:")
    file_names = systemfunc("ls "+file_name_pattern)

    wks = gsn_open_wks("newpdf", "density")

    gsn_define_colormap(wks, "rainbow")

    do l = 0, dimsizes(file_names)-1
        notice("Plotting "+file_names(l)+" ...")

        f = addfile(file_names(l), "r")

        total_mass = 0.0d0
        do j = 0, dimsizes(f->lat)-1
            do i = 0, dimsizes(f->lon)-2
                total_mass = total_mass+f->cell_area(j,i)*f->q(j,i)
            end do
        end do
        notice("Total mass is "+sprintf("%30.20f", total_mass))

        res = True
        ;res@cnFillMode = "RasterFill"
        ;res@gsnPolar   = "NH"
        ;res@mpMinLatF  = 80.0
        setup_contour(res)
        set_contour_range(res, 0.0, 1.1, 0.1)

        plot = gsn_csm_contour_map(wks, f->q, res)

        frame(wks)

    end do

end
