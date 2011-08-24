load "$NCARG_ROOT/lib/ncarg/nclscripts/csm/gsn_code.ncl"
load "$NCARG_ROOT/lib/ncarg/nclscripts/csm/gsn_csm.ncl"
load "$NCARG_ROOT/lib/ncarg/nclscripts/csm/contributed.ncl"

load "/Users/dongli/Works/TTS/Tools/tts_plot_utils.ncl"

begin

    file_names = systemfunc("ls debug_counters00023.nc")

    projection = "CE"

    res_map = setup_map(projection)

    do i = 0, dimsizes(file_names)-1
        system("echo plotting "+file_names(i))
        wks = gsn_open_wks("pdf", systemfunc("basename -s .nc "+file_names(i)))
        map = gsn_csm_map(wks, res_map)
        plot = show_point_counters(wks, file_names(i))
        overlay(map, plot)
        draw(map)
        frame(wks)
        delete(wks)
    end do

end
