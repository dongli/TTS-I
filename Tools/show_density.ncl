load "$NCARG_ROOT/lib/ncarg/nclscripts/csm/gsn_code.ncl"
load "$NCARG_ROOT/lib/ncarg/nclscripts/csm/gsn_csm.ncl"
load "$NCARG_ROOT/lib/ncarg/nclscripts/csm/contributed.ncl"

load "/Users/dongli/Works/TTS/Tools/tts_plot_utils.ncl"

begin

    file_name_pattern = ask("Input file name pattern:")
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
    density_name = "q1_mesh"
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
            density_name = ask("Which one to plot?")
            if (any(density_names .eq. density_name)) then
                break
            else
                report_warning("Please input a valid name!")
            end if
        end do
    end if

    do l = 0, dimsizes(file_names)-1
        notice("Plotting "+file_names(l)+" ...")

        wks = gsn_open_wks("newpdf", systemfunc("basename -s .nc "+ \
            file_names(l))+"_"+density_name+"_density")

        gsn_define_colormap(wks, "WhBlGrYeRe")

        f = addfile(file_names(l), "r")

        total_mass = 0.0d0
        do j = 0, dimsizes(f->lat)-1
            do i = 0, dimsizes(f->lon)-2
                total_mass = total_mass+f->area_mesh(j,i)*f->$density_name$(j,i)
            end do
        end do
        notice("Total mass is "+sprintf("%30.20f", total_mass))

        res = True
        ;res@cnFillMode = "RasterFill"
        ;res@gsnPolar   = "NH"
        ;res@mpMinLatF  = 80.0
        setup_contour(res)
        set_contour_range(res, 0.0, 1.1, 0.1)

        plot = gsn_csm_contour_map(wks, f->$density_name$, res)

        frame(wks)

    end do

end
