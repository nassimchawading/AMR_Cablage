r"""Part 8.6 - small window to send the robot on errands by hand (Tkinter: Python standard library).

    app = OrderApp(sim, mgr, script)       # sim = Factory (or LiveView), script = shift.ScriptedOrders
    sim.tickers.append(app.tick)           # refreshed between simulation steps (same program, no thread)

What the operator can do:
  - NEW ORDER: what to take from the Magasin (a reel colour = its coded slot, or "any reels"), where to
    bring it (Coupe, Sertissage, Assemblage, Expedition; place "auto" = first free place, or a coded
    place 41/42, 46/47), URGENT or not (urgent = priority class 2, before the normal orders);
  - CANCEL a waiting order (selected in the queue);
  - PAUSE / RESUME the scripted reel orders (the operator takes over);
  - SEND THE ROBOT TO CHARGE (after its current order);
  - see what the robot does, its battery, the queue (with the priority class) and the last orders.
Manual orders go through the same mission manager as all the others: same slot choice, same code
check before docking, same priorities. Pickups are limited to the Magasin (material errands): the QC
cell already creates its own orders for its totes.
Closing the window only closes the app: the shift goes on.
"""

import tkinter as tk
from tkinter import ttk

import layout
from missions import Order

REFRESH_EVERY = 4                     # simulation steps between two refreshes (0.2 s simulated)
DROPS = ["coupe", "sertissage", "assemblage", "expedition"]
CLASS_NAME = {1: "batterie", 2: "urgent", 3: "normal"}


def _pickup_choices():
    """Labels -> (what, slot code or None)."""
    out = {f"{what} (emplacement {code})": (what, code) for code, what in layout.INVENTORY.items()}
    out["bobines (n'importe quel emplacement)"] = ("bobines", None)
    return out


class OrderApp:
    def __init__(self, sim, mgr, script=None, title="Ordres AMR - atelier de cablage"):
        self.sim, self.mgr, self.script = sim, mgr, script
        self.n = 0
        self.alive = True
        self.pickups = _pickup_choices()
        self.root = root = tk.Tk()
        root.title(title)
        root.protocol("WM_DELETE_WINDOW", self.close)
        root.geometry("720x640")
        pad = dict(padx=6, pady=3)

        # ---- status
        top = ttk.LabelFrame(root, text="Robot")
        top.pack(fill="x", **pad)
        self.status = tk.StringVar()
        ttk.Label(top, textvariable=self.status, font=("Segoe UI", 11, "bold")).pack(anchor="w", **pad)
        self.battery = ttk.Progressbar(top, maximum=100, length=300)
        self.battery.pack(anchor="w", **pad)
        self.battery_txt = tk.StringVar()
        ttk.Label(top, textvariable=self.battery_txt).pack(anchor="w", padx=6)

        # ---- new order
        new = ttk.LabelFrame(root, text="Nouvel ordre (prise au Magasin)")
        new.pack(fill="x", **pad)
        self.pick = tk.StringVar(value=next(iter(self.pickups)))
        self.drop = tk.StringVar(value=DROPS[0])
        self.place = tk.StringVar(value="auto")
        self.urgent = tk.BooleanVar(value=False)
        ttk.Label(new, text="Quoi").grid(row=0, column=0, sticky="w", **pad)
        ttk.Combobox(new, textvariable=self.pick, values=list(self.pickups), state="readonly",
                     width=40).grid(row=0, column=1, columnspan=3, sticky="w", **pad)
        ttk.Label(new, text="Vers").grid(row=1, column=0, sticky="w", **pad)
        drop_box = ttk.Combobox(new, textvariable=self.drop, values=DROPS, state="readonly", width=14)
        drop_box.grid(row=1, column=1, sticky="w", **pad)
        drop_box.bind("<<ComboboxSelected>>", lambda e: self._update_places())
        ttk.Label(new, text="Place").grid(row=1, column=2, sticky="e", **pad)
        self.place_box = ttk.Combobox(new, textvariable=self.place, state="readonly", width=8)
        self.place_box.grid(row=1, column=3, sticky="w", **pad)
        self._update_places()
        ttk.Checkbutton(new, text="urgent", variable=self.urgent).grid(row=2, column=1, sticky="w", **pad)
        ttk.Button(new, text="Ajouter l'ordre", command=self.add_order).grid(row=2, column=3, sticky="e", **pad)

        # ---- commands
        cmd = ttk.Frame(root)
        cmd.pack(fill="x", **pad)
        self.pause_txt = tk.StringVar(value="Pause des ordres scriptes")
        ttk.Button(cmd, textvariable=self.pause_txt, command=self.toggle_script,
                   state="normal" if script is not None else "disabled").pack(side="left", **pad)
        ttk.Button(cmd, text="Envoyer en charge", command=self.send_to_charge).pack(side="left", **pad)
        ttk.Button(cmd, text="Annuler l'ordre selectionne", command=self.cancel_selected).pack(side="left", **pad)

        # ---- queue + last orders
        q = ttk.LabelFrame(root, text="File d'attente")
        q.pack(fill="both", expand=True, **pad)
        self.queue = tk.Listbox(q, height=8, font=("Consolas", 9))
        self.queue.pack(fill="both", expand=True, **pad)
        f = ttk.LabelFrame(root, text="Derniers ordres")
        f.pack(fill="both", expand=True, **pad)
        self.done = tk.Listbox(f, height=7, font=("Consolas", 9))
        self.done.pack(fill="both", expand=True, **pad)
        self.msg = tk.StringVar()
        ttk.Label(root, textvariable=self.msg, foreground="#205080").pack(anchor="w", **pad)
        self._queue_orders = []
        self.refresh()

    # ------------------------------------------------------------ actions
    def _update_places(self):
        codes = [str(c) for c in layout.spots(self.drop.get()) if c in layout.SLOTS]
        self.place_box["values"] = ["auto"] + codes
        self.place.set("auto")

    def add_order(self):
        what, slot = self.pickups[self.pick.get()]
        place = self.place.get()
        o = Order(what, "magasin", self.drop.get(), source="manual", pickup_slot=slot,
                  drop_slot=None if place == "auto" else int(place), urgent=self.urgent.get())
        self.mgr.add(o)
        self.msg.set(f"ordre #{o.id} ajoute{' (URGENT)' if o.urgent else ''}")
        self.refresh()
        return o

    def toggle_script(self):
        if self.script is None:
            return
        self.script.paused = not self.script.paused
        self.pause_txt.set("Reprendre les ordres scriptes" if self.script.paused else "Pause des ordres scriptes")
        self.msg.set("ordres scriptes en pause" if self.script.paused else "ordres scriptes repris")
        self.mgr.log(f"[app] scripted orders {'PAUSED' if self.script.paused else 'RESUMED'}")

    def send_to_charge(self):
        self.mgr.charge_requested = True
        self.msg.set("recharge demandee (apres l'ordre en cours)")
        self.mgr.log("[app] charge requested")

    def cancel_selected(self):
        sel = self.queue.curselection()
        if not sel:
            self.msg.set("choisir un ordre dans la file d'attente")
            return
        o = self._queue_orders[sel[0]]
        if self.mgr.cancel(o):
            self.msg.set(f"ordre #{o.id} annule")
        else:
            self.msg.set(f"ordre #{o.id} deja commence: il ne peut plus etre annule")
        self.refresh()

    # ------------------------------------------------------------ display
    def refresh(self):
        mgr, sim = self.mgr, self.sim
        t = sim.getSimulationTime()
        cur = mgr.current
        act = {"IDLE": "en attente d'un ordre", "GO_CHARGE": "va a la borne de recharge",
               "CHARGE": "en charge"}.get(mgr.activity, mgr.activity)
        self.status.set(f"{int(t // 60):02d}:{int(t % 60):02d}   "
                        + (act if cur is None else f"#{cur.id} {cur.what}: {cur.pickup} -> {cur.drop} ({cur.state})"))
        bat = getattr(sim, "battery", None)
        if bat is not None:
            self.battery["value"] = bat.level
            self.battery_txt.set(f"batterie {bat.level:.1f} %" + ("  - en charge" if bat.charging else "")
                                 + ("  - recharge demandee" if mgr.charge_requested else ""))
        order = sorted(mgr.waiting, key=lambda o: (mgr.priority(o), o.created, o.id))
        if [o.id for o in order] != [o.id for o in self._queue_orders] or self.n % 20 == 0:
            sel = self.queue.curselection()
            self.queue.delete(0, "end")
            for o in order:
                ready = "" if mgr.ready(o) else "  (pas pret)"
                self.queue.insert("end", f"[{CLASS_NAME.get(mgr.priority(o), '?'):7s}] #{o.id:<3d} {o.source:6s} "
                                         f"{o.what}: {o.pickup} -> {o.drop}"
                                         f"{'[' + str(o.drop_slot) + ']' if o.drop_slot else ''}  {t - o.created:4.0f} s{ready}")
            self._queue_orders = order
            for i in sel:
                if i < len(order):
                    self.queue.selection_set(i)
        if self.done.size() != min(10, len(mgr.finished)) or self.n % 20 == 0:
            self.done.delete(0, "end")
            for o in mgr.finished[-10:][::-1]:
                self.done.insert("end", f"#{o.id:<3d} {o.state:9s} {o.source:6s} {o.what}: "
                                        f"{o.pickup_slot} -> {o.drop_slot}  {o.done - o.started:4.0f} s")

    def tick(self):
        """Called before every simulation step (Factory.tickers): refresh + handle the clicks."""
        if not self.alive:
            return
        self.n += 1
        if self.n % REFRESH_EVERY:
            return
        try:
            self.refresh()
            self.root.update()
        except tk.TclError:              # window closed
            self.alive = False

    def close(self):
        self.alive = False
        try:
            self.root.destroy()
        except tk.TclError:
            pass
