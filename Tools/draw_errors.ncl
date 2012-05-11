load "$NCARG_ROOT/lib/ncarg/nclscripts/csm/gsn_code.ncl"
load "$NCARG_ROOT/lib/ncarg/nclscripts/csm/gsn_csm.ncl"
load "$NCARG_ROOT/lib/ncarg/nclscripts/csm/contributed.ncl"

load "/Users/dongli/Works/TTS/Tools/tts_plot_utils.ncl"

begin

    if (.not. isdefined("file_name_pattern1")) then
        file_name_pattern1 = ask("Input the true solution file name pattern:")
        file_names1 = systemfunc("ls "+file_name_pattern1)
    end if
    if (.not. isdefined("file_name_pattern2")) then
        file_name_pattern2 = ask("Input the polygon file name pattern:")
        file_names2 = systemfunc("ls "+file_name_pattern2)
    end if

    f1 = addfile(file_names1(0), "r")

    num_lon = dimsizes(f1->lon)
    num_lat = dimsizes(f1->lat)
    num_time = dimsizes(file_names1)

    times = new(num_time, double)
    errors = new((/3,num_time/), double)

    do i = 0, num_time-1
        times(i) = i

        f1 = addfile(file_names1(i), "r")
        f2 = addfile(file_names2(i), "r")

        q1 = f1->q0_mesh
        q2 = f2->q0_mesh
        area = f1->area_mesh

        dq = abs(q2-q1)
        I1_1 = sum(dq*area)
        I1_2 = sum(abs(q1)*area)
        I2_1 = sum(dq^2.0*area)
        I2_2 = sum(q1^2.0*area)

        errors(0,i) = I1_1/I1_2
        errors(1,i) = sqrt(I2_1/I2_2)
        errors(2,i) = max(abs(dq))/max(abs(q1))

    end do

    wks = gsn_open_wks("pdf", "errors")

    res = True
    res@gsnMaximize = True
    res@tiXAxisString = "Time (Days)"
    res@tiYAxisString = "Normalized errors"
    res@trYMaxF = 0.1
    res@trYMinF = 0.0
    res@pmLegendDisplayMode = "Always"
    res@pmLegendSide = "Top"
    res@pmLegendParallelPosF = 0.2
    res@pmLegendOrthogonalPosF = -0.4
    res@pmLegendWidthF = 0.2
    res@pmLegendHeightF = 0.15
    res@lgPerimOn = False
    res@lgLabelFontHeightF = .02
    res@xyExplicitLegendLabels = (/"L1","L2","LInf"/)
    res@xyLineThicknesses = (/3.,3.,3./)

    plot = gsn_csm_xy(wks, times, errors, res)

end
