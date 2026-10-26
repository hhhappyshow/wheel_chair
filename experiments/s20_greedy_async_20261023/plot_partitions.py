"""Three-view layouts for the current-arrival geometry-only zoning comparison."""
import argparse
import json
from pathlib import Path
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle


def main():
    ap=argparse.ArgumentParser();ap.add_argument("--folder",type=Path,required=True);args=ap.parse_args()
    reports=json.loads((args.folder/"comparison.json").read_text())
    fig,axes=plt.subplots(3,3,figsize=(13,10),constrained_layout=True)
    for i,(name,data) in enumerate(reports.items()):
        h=data["hull"];length=h["length"];width=h["width"]
        for j,(a,b,xmax,ymax) in enumerate(((1,2,width,1.),(0,1,length,width),(0,2,length,1.))):
            ax=axes[i,j];ax.set(xlim=(-.03,xmax+.03),ylim=(-.03,ymax+.03),aspect="equal")
            ax.add_patch(Rectangle((0,0),xmax,ymax,fill=False,lw=1.5,color="black"))
            ax.set_title(f"{name} | {'front / YZ' if j==0 else 'top / XY' if j==1 else 'side / XZ'}")
            ax.set_xlabel("XYZ"[a]+" (m)");ax.set_ylabel("XYZ"[b]+" (m)")
            labels=[]
            for step in data["steps"]:
                if "box" not in step:continue
                box=step["box"];pos=box["pos"];dims=box["dims"];is24=step["source_dimensions_m"][0]>.6
                color="#e9a342" if is24 else "#54aeb3"
                ax.add_patch(Rectangle((pos[a],pos[b]),dims[a],dims[b],facecolor=color,edgecolor="white",alpha=.7))
                cx,cy=pos[a]+dims[a]/2,pos[b]+dims[b]/2
                for item in labels:
                    if abs(item[0]-cx)<.09 and abs(item[1]-cy)<.07:
                        item[2].append(step["id"][1:]);break
                else:labels.append([cx,cy,[step["id"][1:]]])
            for cx,cy,ids in labels:ax.text(cx,cy,"/".join(ids),ha="center",va="center",fontsize=8)
            ax.grid(alpha=.15)
    fig.suptitle("Online greedy / geometry-only: 1.70m endpoint sphere, 1.00m box-top limit\n20in=55x40x25cm; 24in=65x45x30cm; proxy sizes, no arm-path certificate",fontsize=12)
    fig.savefig(args.folder/"partition_three_views.png",dpi=150);plt.close(fig)


if __name__=="__main__":main()
