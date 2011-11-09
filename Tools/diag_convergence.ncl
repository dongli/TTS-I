load "$NCARG_ROOT/lib/ncarg/nclscripts/csm/gsn_code.ncl"
load "$NCARG_ROOT/lib/ncarg/nclscripts/csm/gsn_csm.ncl"
load "$NCARG_ROOT/lib/ncarg/nclscripts/csm/contributed.ncl"

load "/Users/dongli/Works/TTS/Tools/tts_plot_utils.ncl"

begin

    file_name0 = ask("Input the initial condition file name:")
    file_name1 = ask("Input the final result file name:")

    f0 = addfile(file_name0, "r")
    f1 = addfile(file_name1, "r")

    num_lon = dimsizes(f0->lon)-1
    num_lat = dimsizes(f0->lat)

    L_2 = new(1, double)
    L_inf = new(1, double)

    q1_0 = f0->q1_mesh(:,:num_lon-1)/f0->q0_mesh(:,:num_lon-1)
    q1_1 = f1->q1_mesh(:,:num_lon-1)/f1->q0_mesh(:,:num_lon-1)
    area = f0->area_mesh(:,:num_lon-1)

    I1 = sum((q1_1-q1_0)^2.0*area)/4.0d0/PI
    I2 = sum(q1_0^2.0*area)/4.0d0/PI

    L_2 = sqrt(I1/I2)
    L_inf = max(q1_1-q1_0)/max(q1_0)

    notice("L_2 normalized error is "+sprintf("%5.3E", L_2))
    notice("L_inf normalized error is "+sprintf("%5.3E", L_inf))
    notice("Maximum mixing ratio is "+sprintf("%5.3E", max(q1_1)))
    notice("Minimum mixing ratio is "+sprintf("%5.3E", min(q1_1)))

end
