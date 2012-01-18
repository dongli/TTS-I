load "$NCARG_ROOT/lib/ncarg/nclscripts/csm/gsn_code.ncl"
load "$NCARG_ROOT/lib/ncarg/nclscripts/csm/gsn_csm.ncl"
load "$NCARG_ROOT/lib/ncarg/nclscripts/csm/contributed.ncl"

load "/Users/dongli/Works/TTS/Tools/tts_plot_utils.ncl"

begin

    file_name_pattern = ask("Input file name pattern:")
    file_names = systemfunc("ls "+file_name_pattern)

    ; --------------------------------------------------------------------------
    ; get tracer mixing ratio variable names
    f = addfile(file_names(0), "r")
    var_names = getfilevarnames(f)
    num_tracer = 0
    do l = 0, dimsizes(var_names)-1
        if (.not. ismissing(str_match(var_names(l), "q.*_mesh"))) then
            num_tracer = num_tracer+1
        end if
    end do
    mr_name = "q1_mesh"
    if (num_tracer .gt. 2) then
        mr_names = new(num_tracer-1, string)
        i = 0
        do l = 0, dimsizes(var_names)-1
            if (.not. ismissing(str_match(var_names(l), "q.*_mesh"))) then
                if (var_names(l) .ne. "q0_mesh") then
                    mr_names(i) = var_names(l)
                    i = i+1
                end if
            end if
        end do

        ; ask user which tracer is going to be plotted
        notice("There are "+sprinti("%d", num_tracer)+" tracers:")
        do i = 0, num_tracer-1
            system("echo  '  * "+mr_names(i)+"'")
        end do
        do while (True)
            mr_name = ask("Which one to plot?")
            if (any(mr_names .eq. mr_name)) then
                break
            else
                report_warning("Please input a valid name!")
            end if
        end do
    end if

    do l = 0, dimsizes(file_names)-1
        notice("Plotting "+file_names(l)+" ...")

        wks = gsn_open_wks("newpdf", systemfunc("basename -s .nc "+ \
            file_names(l))+"_"+mr_name+"_mr")

        gsn_define_colormap(wks, "WhBlGrYeRe")

        f = addfile(file_names(l), "r")

        res = True
        res@mpOutlineOn = False
        setup_contour(res)
        set_contour_range(res, 0.0, 1.1, 0.1)

        mr = f->$mr_name$/f->q0_mesh
        copy_VarCoords(f->$mr_name$, mr)

        res@gsnStringFontHeightF = 0.02
        res@gsnRightString = "min: "+sprintf("%4.2f", min(mr))+", "+ \
                             "max: "+sprintf("%4.2f", max(mr))
        plot = gsn_csm_contour_map(wks, mr, res)

        frame(wks)

    end do

end
