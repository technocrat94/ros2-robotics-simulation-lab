"""Shared v4 prediction graphics; geometry uses recorded estimates only."""
import numpy as np


class PredictionVisuals:
    def __init__(self,server,floor_top,*,floor_thickness=.1):
        self.server=server
        server.scene.add_box('/truth/floor',dimensions=(10.,10.,floor_thickness),position=(0.,0.,floor_top-floor_thickness/2),color=(120,130,140),opacity=.18)
        server.gui.add_markdown('All groups retain the floor; A has no rod obstacle. **Legend:** gray = floor; orange = observed/remembered box; blue = swept MPC box; cyan ghost = estimate at +0.43 s; green arrow = estimated velocity. Display toggle does not change control.')
        self.visible=server.gui.add_checkbox('Show prediction',initial_value=True)
        self.ghost=server.scene.add_cylinder('/prediction/ghost',radius=.025,height=.25,color=(40,235,235),opacity=.35,visible=False)
        self.arrow=server.scene.add_arrows('/prediction/velocity',points=np.zeros((1,2,3),dtype=np.float32),colors=(40,220,90),shaft_radius=.003,head_radius=.009,head_length=.008,visible=False)
        self.speed=server.gui.add_text('Estimated / true rod speed','',disabled=True)
        self.enabled=server.gui.add_text('Controller prediction','',disabled=True)
        self.last=None
        self.visible.on_update(lambda _event:self.refresh())
        self.box=None

    def update(self,fit,estimate,true_velocity,horizon,prediction,predicted_box):
        self.last=(fit,np.asarray(estimate),np.asarray(true_velocity),horizon,prediction)
        self.box=predicted_box
        self.refresh()

    def refresh(self):
        if self.last is None:return
        fit,estimate,truth,horizon,prediction=self.last
        self.speed.value=(f'{np.linalg.norm(estimate):.3f} / {np.linalg.norm(truth):.3f} m/s; X {estimate[0]:+.3f} / {truth[0]:+.3f}' if fit is not None else f'unavailable / {np.linalg.norm(truth):.3f} m/s (no box)')
        self.enabled.value='on' if prediction else 'off'
        show=bool(self.visible.value and prediction and fit is not None)
        self.ghost.visible=show
        self.arrow.visible=bool(show and float(np.linalg.norm(estimate))>1e-9)
        if self.box is not None:self.box.visible=show
        if fit is not None:
            center=np.asarray(fit['center_m'])
            self.ghost.position=tuple(center+estimate*horizon)
            self.arrow.points=np.array([[center,center+estimate*horizon]],dtype=np.float32)
