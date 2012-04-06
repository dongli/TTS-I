load "$NCARG_ROOT/lib/ncarg/nclscripts/csm/gsn_code.ncl"
load "$NCARG_ROOT/lib/ncarg/nclscripts/csm/gsn_csm.ncl"
load "$NCARG_ROOT/lib/ncarg/nclscripts/csm/contributed.ncl"

load "$GEODIAG_UTILS/geodiag_plot_utils.ncl"

begin

    projection = "CE"
    ;projection@center_lon = 0.0
    ;projection@center_lat = 90.0
    projection@angle = 10.0

    notice("Input file name pattern:")
    file_name_pattern = get_answer()
    file_names = systemfunc("ls "+file_name_pattern)

    ; --------------------------------------------------------------------------
    ; get tracer density variable names
    f = addfile(file_names(0), "r")
    var_names = getfilevarnames(f)
    num_tracer = 0
    do l = 0, dimsizes(var_names)-1
        if (.not. ismissing(str_match(var_names(l), "q.*_mesh"))) then
            num_tracer = num_tracer+1
        end if
    end do
    density_name = "q0_mesh"
    if (num_tracer .gt. 1) then
        density_names = new(num_tracer, string)
        i = 0
        do l = 0, dimsizes(var_names)-1
            if (.not. ismissing(str_match(var_names(l), "q.*_mesh"))) then
                density_names(i) = var_names(l)
                i = i+1
            end if
        end do

        ; ask user which tracer is going to be plotted
        notice("There are "+sprinti("%d", num_tracer)+" tracers:")
        do i = 0, num_tracer-1
            system("echo  '  * "+density_names(i)+"'")
        end do
        do while (True)
            notice("Which one to plot?")
            density_name = get_answer()
            if (any(density_names .eq. density_name)) then
                break
            else
                report_warning("Please input a valid name!")
            end if
        end do
    end if

    do l = 0, dimsizes(file_names)-1
        notice("Plotting "+file_names(l)+" ...")

        wks = gsn_open_wks("pdf", systemfunc("basename -s .nc "+ \
            file_names(l))+"_"+density_name+"_density")

        ;gsn_define_colormap(wks, "wh-bl-gr-ye-re")
        gsn_define_colormap(wks, "BlueYellowRed")
        ;gsn_define_colormap(wks, "WhBlGrYeRe")
        ;gsn_define_colormap(wks, "WhBlReWh")

        f = addfile(file_names(l), "r")

        res = True
        res@lbLabelAngleF = 45.0
        res@gsnAddCyclic = False
        res@pmLabelBarOrthogonalPosF = 0.1
        res@gsnMaximize = True

        setup_map(projection, res)
        setup_contour(res)
        res@gsnMaximize = True

        ; for GAMIL test
        ranges = fspan(1.0e-6, 2.0e-2, 21)
        set_contour_range(res, ranges, "%7.5g")
        res@lbLabelAngleF = 45.0
        res@pmLabelBarOrthogonalPosF = 0.15

        ; for deformation flow test
        ;set_contour_range(res, (/0.0,1.1,0.1/), "%4.2f")

        ;set_contour_range(res, 0.5, 1.5, 0.05, "%4.2f")
        ;set_contour_range(res, 0.0, 1000.0, 100.0, "%8.2f")

        res@gsnRightString = "min: "+sprintf("%4.2f", min(f->$density_name$))+", "+ \
                             "max: "+sprintf("%4.2f", max(f->$density_name$))
        plot = gsn_csm_contour_map(wks, f->$density_name$, res)

        draw(plot)
        frame(wks)
        delete(wks)
    end do

end
