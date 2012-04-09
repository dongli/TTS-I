load "$NCARG_ROOT/lib/ncarg/nclscripts/csm/gsn_code.ncl"
load "$NCARG_ROOT/lib/ncarg/nclscripts/csm/gsn_csm.ncl"
load "$NCARG_ROOT/lib/ncarg/nclscripts/csm/contributed.ncl"

load "$GEODIAG_UTILS/geodiag_plot_utils.ncl"

begin

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

    total_mass = new(dimsizes(file_names), double)
    total_mass = 0.0d0

    f = addfile(file_names(0), "r")
    idx_j = ispan(0, dimsizes(f->lat)-1, 1)
    idx_i = ispan(0, dimsizes(f->lon)-1, 1)

    do l = 0, dimsizes(file_names)-1
        f = addfile(file_names(l), "r")

        total_mass(l) = sum(f->area_mesh(idx_j,idx_i)* \
                            f->$density_name$(idx_j,idx_i))
        notice("time step "+l+" total mass "+sprintf("%20.16f", total_mass(l)))
    end do

    time_steps = ispan(0, dimsizes(file_names)-1, 1)

    wks = gsn_open_wks("pdf", "total_mass")

    total_mass = total_mass/1.0e17

    res = True
    setup_xy(res, "Time Step", (/0, 2100, 300/), \
                  "Total Mass", (/3.8528,3.8530,1.e-4/))

    plot = gsn_csm_xy(wks, time_steps, total_mass, res)

    draw(plot)
    frame(wks)

end
