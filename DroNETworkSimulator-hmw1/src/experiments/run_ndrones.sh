#-----------------------------------------------------------#
#           _  _ ___  ___  ___  _  _ ___ ___                #
#          | \| |   \| _ \/ _ \| \| | __/ __|               #
#          | .` | |) |   / (_) | .` | _|\__ \               #
#          |_|\_|___/|_|_\\___/|_|\_|___|___/               #
#                                                           #
#-----------------------------------------------------------#

#test baselines
for nd in "5" "10" "15" "30";
do
    for alg in  "AI" ;
    do 
        echo "run: ${alg} - ndrones ${nd} "
        #python3 -m src.experiments.experiment_ndrones -nd ${nd} -i_s 1 -e_s 3 -alg ${alg} &
        python -m src.experiments.experiment_ndrones -nd ${nd} -i_s 10 -e_s 20 -alg ${alg} 
        #python -m src.experiments.experiment_ndrones -nd ${nd} -i_s 20 -e_s 30 -alg ${alg} 
    done;
done; 
wait

#python -m src.experiments.json_and_plot -nd 5 -nd 10 -nd 15 -nd 30 -i_s 20 -e_s 30 -exp_suffix GEO -exp_suffix RND -exp_suffix AI

