#!/usr/bin/env bash

######### DIRECTORIES & FILES #################################################
DIR=/home/pulver/Desktop/loop_closure
RESULTS_DIR=/tmp/test_ndt
mkdir -p $RESULTS_DIR
SESSION_ID=(kg_lc_march
            kg_lc_april
            kg_lc_may
            kg_lc_june)
len_session_list=${#SESSION_ID[@]}
MAPPING=fast_lio  # fast_lio, hdl

######### TOPICS & LOCALIZER ##############################################################
GPS_TOPIC=/odometry/gps
NDT_ODOM=/ndt_pose
if [ "$MAPPING" == "fast_lio" ]; then
    MAP_ODOM=/Odometry  # /odom from hdl, /Odometry for fast_lio
else
    MAP_ODOM=/odom
fi
POINTS_TOPIC=/os_cloud_node/points_filtered
NDT_PUBLISH_TF=true

######### VARIES ##############################################################
DELAY=40.0
VOXEL_SIZE=0.1
batch_size=1  # how many experiments to launch in parallel
counter=0
i=1

######### EVO configs #########################################################
evo_config set save_traj_in_zip true
# evo_config set plot_export_format eps
evo_config set plot_figsize  12 12 
evo_config set plot_seaborn_style whitegrid
evo_config set plot_usetex false
# evo_config set plot_texsystem "pdflatex"
evo_config set plot_pose_correspondences false

USE_TUM=true

###############################################################################

for ((map_id=0; map_id<1; map_id++))
do
    for ((bag_id=3; bag_id<4; bag_id++))
    do
        echo "[$map_id] Localizing on map: ${DIR}/pcd/${SESSION_ID[$map_id]}.pcd"
        echo "[$bag_id] using rosbag: ${DIR}/bags/slam_traj_${SESSION_ID[$bag_id]}.bag"
        TMP_DIR=${RESULTS_DIR}/${SESSION_ID[$map_id]}
        mkdir -p ${TMP_DIR}
        EXPERIMENT_PATH=${TMP_DIR}/traj_${SESSION_ID[$map_id]}_${SESSION_ID[$bag_id]}  #  e.g, traj_ktima_march_ktima_may
        
        echo "  1) Call roslaunch passing the PATH and the ROSBAG for testing NDT-Localizer"
        if test -f "${EXPERIMENT_PATH}.bag"; then
            echo "${EXPERIMENT_PATH}.bag exists."
        else
            roslaunch bacchus_localisation ndt_localizer_experiments.launch pcd_path:=${DIR}/pcd/${SESSION_ID[$map_id]}.pcd bag_filepath:=${DIR}/bags/slam_traj_${SESSION_ID[$bag_id]}.bag mapping_topic:=${MAP_ODOM} output_bag_name:=${EXPERIMENT_PATH} points_topic:=${POINTS_TOPIC} publish_tf:=${NDT_PUBLISH_TF} leaf_size:=${VOXE_SIZE} play_delay:=${DELAY} &
            sleep 180 # Need to give time to NDT-localizer to produce output
            rosnode kill -a # kill all the nodes 
            sleep 10
        fi
        
        echo "  2) Call evo for computing localization error vs ground truth offered by the GPS signal"
        
        if $USE_TUM
        then
            evo_traj bag ${EXPERIMENT_PATH}.bag $NDT_ODOM --save_as_tum #--ref $GPS_TOPIC --plot_mode xy --save_plot ${TMP_DIR}/traj_${SESSION_ID[$map_id]}_${SESSION_ID[$bag_id]}_loc.pdf --align #--n_to_align 5000  
            # rm odometry_gps.tum
            evo_traj bag ${EXPERIMENT_PATH}.bag $MAP_ODOM --save_as_tum #--ref $GPS_TOPIC --plot_mode xy --save_plot ${TMP_DIR}/traj_${SESSION_ID[$map_id]}_${SESSION_ID[$bag_id]}_map.pdf --align #--n_to_align 5000 
            mv Odometry.tum MAP_${SESSION_ID[$bag_id]}.tum
            evo_traj bag ${EXPERIMENT_PATH}.bag $GPS_TOPIC --save_as_tum
            mv odometry_gps.tum GPS.tum
            mv ndt_pose.tum NDT_${SESSION_ID[$bag_id]}.tum 
            GPS_TS=$(cat GPS.tum | { read first rest ; echo $first ; })
            MAP_TS=$(cat MAP_${SESSION_ID[$bag_id]}.tum | { read first rest ; echo $first ; })
            ODOM_TS=$(cat NDT_${SESSION_ID[$bag_id]}.tum | { read first rest ; echo $first ; })
            GPS_TS=$(echo $GPS_TS | sed 's/\([0-9]*\(\.[0-9]*\)\?\)[eE]+\?\(-\?[0-9]*\)/(\1*10^\3)/g;s/^/scale=30;/'| bc)
            MAP_TS=$(echo $MAP_TS | sed 's/\([0-9]*\(\.[0-9]*\)\?\)[eE]+\?\(-\?[0-9]*\)/(\1*10^\3)/g;s/^/scale=30;/'| bc)
            ODOM_TS=$(echo $ODOM_TS | sed 's/\([0-9]*\(\.[0-9]*\)\?\)[eE]+\?\(-\?[0-9]*\)/(\1*10^\3)/g;s/^/scale=30;/'| bc)

            echo "GPS_TS: $MAP_TS"
            echo "MAP_TS: $MAP_TS"
            echo "ODOM_TS: $ODOM_TS"
            TS_OFFSET=$(echo "$GPS_TS-$ODOM_TS" | bc -l)
            echo "Diff: $TS_OFFSET"
        else
            evo_traj bag ${EXPERIMENT_PATH}.bag $NDT_ODOM --ref $GPS_TOPIC --plot_mode xy --save_plot ${EXPERIMENT_PATH}_results_ndt.pdf --t_max_diff 0.1 --align --full_check >> ${EXPERIMENT_PATH}_results.txt
            evo_traj bag ${EXPERIMENT_PATH}.bag $MAP_ODOM --ref $GPS_TOPIC --plot_mode xy --save_plot ${EXPERIMENT_PATH}_results_lio.pdf --t_max_diff 0.1 --align --full_check >> ${EXPERIMENT_PATH}_results.txt
        fi
        
        echo "  3) Compute absolute and relative pose error"
        if $USE_TUM
        then
            if [ "$map_id" == "$bag_id" ]  # The MAP odometry must be calculated only during the mapping phase (bag and maps of the same session)
            then
                evo_rpe tum GPS.tum MAP_${SESSION_ID[$map_id]}.tum --align --t_max_diff 999999999999 --save_results ${EXPERIMENT_PATH}_results_map_rpe.zip >> ${EXPERIMENT_PATH}_results.txt
                evo_ape tum GPS.tum MAP_${SESSION_ID[$map_id]}.tum --align --t_max_diff 999999999999 --save_results ${EXPERIMENT_PATH}_results_map_ape.zip >> ${EXPERIMENT_PATH}_results.txt
            # else
            fi
            # evo_rpe tum MAP_${SESSION_ID[$map_id]}.tum NDT_${SESSION_ID[$bag_id]}.tum --align --t_offset $TS_OFFSET --save_results ${EXPERIMENT_PATH}_results_ndt_rpe.zip >> ${EXPERIMENT_PATH}_results.txt 
            # evo_ape tum MAP_${SESSION_ID[$map_id]}.tum NDT_${SESSION_ID[$bag_id]}.tum --align --t_offset $TS_OFFSET --save_results ${EXPERIMENT_PATH}_results_ndt_ape.zip >> ${EXPERIMENT_PATH}_results.txt
            evo_rpe tum GPS.tum NDT_${SESSION_ID[$bag_id]}.tum --align --t_offset $TS_OFFSET --save_results ${EXPERIMENT_PATH}_results_ndt_rpe.zip >> ${EXPERIMENT_PATH}_results.txt 
            evo_ape tum GPS.tum NDT_${SESSION_ID[$bag_id]}.tum --align --t_offset $TS_OFFSET --save_results ${EXPERIMENT_PATH}_results_ndt_ape.zip >> ${EXPERIMENT_PATH}_results.txt
            # fi
            
        else
            evo_rpe bag ${EXPERIMENT_PATH}.bag $GPS_TOPIC $MAP_ODOM --plot_mode xy --save_plot ${EXPERIMENT_PATH}_map_rpe.pdf --t_max_diff 0.1 --align --save_results ${EXPERIMENT_PATH}_results_map_rpe.zip >> ${EXPERIMENT_PATH}_results.txt
            evo_ape bag ${EXPERIMENT_PATH}.bag $GPS_TOPIC $MAP_ODOM --plot_mode xy --save_plot ${EXPERIMENT_PATH}_map_ape.pdf --t_max_diff 0.1 --align --save_results ${EXPERIMENT_PATH}_results_map_ape.zip >> ${EXPERIMENT_PATH}_results.txt
            evo_rpe bag ${EXPERIMENT_PATH}.bag $GPS_TOPIC $NDT_ODOM --plot_mode xy --save_plot ${EXPERIMENT_PATH}_ndt_rpe.pdf --t_max_diff 0.1 --align --save_results ${EXPERIMENT_PATH}_results_ndt_rpe.zip >> ${EXPERIMENT_PATH}_results.txt
            evo_ape bag ${EXPERIMENT_PATH}.bag $GPS_TOPIC $NDT_ODOM --plot_mode xy --save_plot ${EXPERIMENT_PATH}_ndt_ape.pdf --t_max_diff 0.1 --align --save_results ${EXPERIMENT_PATH}_results_ndt_ape.zip >> ${EXPERIMENT_PATH}_results.txt
        fi
        
        echo "  4) Compute metrics between the localization trajectory and the ground truth"
        if [ "$USE_TUM" == "false" ]
        then
            evo_res ${EXPERIMENT_PATH}_results_map_rpe.zip --save_plot ${EXPERIMENT_PATH}_metrics_map_rpe.pdf --save_table ${EXPERIMENT_PATH}_table_map_rpe.csv >> ${EXPERIMENT_PATH}_results.txt
            evo_res ${EXPERIMENT_PATH}_results_map_ape.zip --save_plot ${EXPERIMENT_PATH}_metrics_map_ape.pdf --save_table ${EXPERIMENT_PATH}_table_map_ape.csv >> ${EXPERIMENT_PATH}_results.txt
            evo_res ${EXPERIMENT_PATH}_results_ndt_rpe.zip --save_plot ${EXPERIMENT_PATH}_metrics_ndt_rpe.pdf --save_table ${EXPERIMENT_PATH}_table_ndt_rpe.csv >> ${EXPERIMENT_PATH}_results.txt
            evo_res ${EXPERIMENT_PATH}_results_ndt_ape.zip --save_plot ${EXPERIMENT_PATH}_metrics_ndt_ape.pdf --save_table ${EXPERIMENT_PATH}_table_ndt_ape.csv >> ${EXPERIMENT_PATH}_results.txt
        fi
        
        echo "   5) Clean the environment"
        if $USE_TUM
        then
            rm NDT_${SESSION_ID[$bag_id]}.tum
            rm GPS.tum
            # rm MAP_${SESSION_ID[$bag_id]}.tum
        fi

        echo ""
    done
    echo "[$map_id] Compute final metrics on the current map"
    evo_res ${TMP_DIR}/*ape.zip --save_plot ${EXPERIMENT_PATH}_final_metrics_ape.pdf --save_table ${EXPERIMENT_PATH}_final_table_ape.csv >> ${TMP_DIR}/final_results.txt
    evo_res ${TMP_DIR}/*rpe.zip --save_plot ${EXPERIMENT_PATH}_final_metrics_rpe.pdf --save_table ${EXPERIMENT_PATH}_final_table_rpe.csv >> ${TMP_DIR}/final_results.txt

done    
echo "Experiments completed!"
