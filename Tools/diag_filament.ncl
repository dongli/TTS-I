load "$NCARG_ROOT/lib/ncarg/nclscripts/csm/gsn_code.ncl"
load "$NCARG_ROOT/lib/ncarg/nclscripts/csm/gsn_csm.ncl"
load "$NCARG_ROOT/lib/ncarg/nclscripts/csm/contributed.ncl"

load "$GEODIAG_UTILS/geodiag_plot_utils.ncl"

begin

    notice("Input the reference data file name:")
    file_name0 = get_answer()
    notice("Input the data file name:")
    file_name1 = get_answer()

    f0 = addfile(file_name0, "r")
    f1 = addfile(file_name1, "r")

    num_lon = dimsizes(f0->lon)-1
    num_lat = dimsizes(f0->lat)

    tau = fspan(0.1, 1.0, 19)

    area0 = new(dimsizes(tau), double)
    area1 = new(dimsizes(tau), double)
    L_f = new(dimsizes(tau), double)

    q1_0 = f0->q1_mesh/f0->q0_mesh
    q1_1 = f1->q1_mesh/f1->q0_mesh
    area = f0->area_mesh

    area0 = 0.0
    area1 = 0.0
    do j = 0, num_lat-1
        do i = 0, num_lon-1
            do k = 0, dimsizes(tau)-1
                if (q1_0(j,i) .ge. tau(k)) then
                    area0(k) = area0(k)+area(j,i)
                end if
                if (q1_1(j,i) .ge. tau(k)) then
                    area1(k) = area1(k)+area(j,i)
                end if
            end do
        end do
    end do

    do k = 0, dimsizes(tau)-1
        if (area0(k) .ne. 0.0) then
            L_f(k) = area1(k)/area0(k)*100.0
        else
            L_f(k) = 0.0
        end if
        system("echo "+sprintf("%4.2f", tau(k))+"    "+sprintf("%6.2f", L_f(k)))
    end do

end
