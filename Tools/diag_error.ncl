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

    num_lon = dimsizes(f0->lon)
    num_lat = dimsizes(f0->lat)

    q0 = f0->q1_mesh/f0->q0_mesh
    q1 = f1->q1_mesh/f1->q0_mesh
    area = f0->area_mesh

    I1_1 = sum(abs(q1-q0)*area)
    I1_2 = sum(abs(q0)*area)
    I2_1 = sum((q1-q0)^2.0*area)
    I2_2 = sum(q0^2.0*area)

    L_1 = I1_1/I1_2
    L_2 = sqrt(I2_1/I2_2)
    L_inf = max(abs(q1-q0))/max(abs(q0))

    notice("L_1 normalized error is "+sprintf("%5.4E", L_1))
    notice("L_2 normalized error is "+sprintf("%5.4E", L_2))
    notice("L_inf normalized error is "+sprintf("%5.4E", L_inf))

end
