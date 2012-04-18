load "$NCARG_ROOT/lib/ncarg/nclscripts/csm/gsn_code.ncl"
load "$NCARG_ROOT/lib/ncarg/nclscripts/csm/gsn_csm.ncl"
load "$NCARG_ROOT/lib/ncarg/nclscripts/csm/contributed.ncl"

load "$GEODIAG_UTILS/geodiag_plot_utils.ncl"

begin

    notice("Input the initial condition file name:")
    file_name0 = get_answer()
    notice("Input the final result file name:")
    file_name1 = get_answer()

    f0 = addfile(file_name0, "r")
    f1 = addfile(file_name1, "r")

    num_lon = dimsizes(f0->lon)-1
    num_lat = dimsizes(f0->lat)

    q1_0 = f0->q1_mesh(:,:num_lon-1)/f0->q0_mesh(:,:num_lon-1)
    q1_1 = f1->q1_mesh(:,:num_lon-1)/f1->q0_mesh(:,:num_lon-1)
    area = f0->area_mesh(:,:num_lon-1)

    I1_1 = sum(fabs(q1_1-q1_0)*area)
    I1_2 = sum(q1_0*area)
    I2_1 = sum((q1_1-q1_0)^2.0*area)
    I2_2 = sum(q1_0^2.0*area)

    L_1 = I1_1/I1_2
    L_2 = sqrt(I2_1/I2_2)
    L_inf = max(q1_1-q1_0)/max(q1_0)

    notice("L_1 normalized error is "+sprintf("%5.4E", L_1))
    notice("L_2 normalized error is "+sprintf("%5.4E", L_2))
    notice("L_inf normalized error is "+sprintf("%5.4E", L_inf))

end
