from rlbench.tasks.basketball_in_hoop import BasketballInHoop
from rlbench.tasks.beat_the_buzz import BeatTheBuzz
from rlbench.tasks.block_pyramid import BlockPyramid
from rlbench.tasks.change_channel import ChangeChannel
from rlbench.tasks.change_clock import ChangeClock
from rlbench.tasks.close_box import CloseBox
from rlbench.tasks.close_door import CloseDoor
from rlbench.tasks.close_drawer import CloseDrawer
from rlbench.tasks.close_fridge import CloseFridge
from rlbench.tasks.close_grill import CloseGrill
from rlbench.tasks.close_jar import CloseJar
from rlbench.tasks.close_laptop_lid import CloseLaptopLid
from rlbench.tasks.close_microwave import CloseMicrowave
from rlbench.tasks.empty_container import EmptyContainer
from rlbench.tasks.empty_dishwasher import EmptyDishwasher
from rlbench.tasks.get_ice_from_fridge import GetIceFromFridge
from rlbench.tasks.hang_frame_on_hanger import HangFrameOnHanger
from rlbench.tasks.hit_ball_with_queue import HitBallWithQueue
from rlbench.tasks.hockey import Hockey
from rlbench.tasks.insert_onto_square_peg import InsertOntoSquarePeg
from rlbench.tasks.insert_usb_in_computer import InsertUsbInComputer
from rlbench.tasks.lamp_off import LampOff
from rlbench.tasks.lamp_on import LampOn
from rlbench.tasks.lift_numbered_block import LiftNumberedBlock
from rlbench.tasks.light_bulb_in import LightBulbIn
from rlbench.tasks.light_bulb_out import LightBulbOut
from rlbench.tasks.meat_off_grill import MeatOffGrill
from rlbench.tasks.meat_on_grill import MeatOnGrill
from rlbench.tasks.move_hanger import MoveHanger
from rlbench.tasks.open_box import OpenBox
from rlbench.tasks.open_door import OpenDoor
from rlbench.tasks.open_drawer import OpenDrawer
from rlbench.tasks.open_fridge import OpenFridge
from rlbench.tasks.open_grill import OpenGrill
from rlbench.tasks.open_jar import OpenJar
from rlbench.tasks.open_microwave import OpenMicrowave
from rlbench.tasks.open_oven import OpenOven
from rlbench.tasks.open_washing_machine import OpenWashingMachine
from rlbench.tasks.open_window import OpenWindow
from rlbench.tasks.open_wine_bottle import OpenWineBottle
from rlbench.tasks.phone_on_base import PhoneOnBase
from rlbench.tasks.pick_and_lift import PickAndLift
from rlbench.tasks.pick_and_lift_small import PickAndLiftSmall
from rlbench.tasks.pick_up_cup import PickUpCup
from rlbench.tasks.place_cups import PlaceCups
from rlbench.tasks.place_hanger_on_rack import PlaceHangerOnRack
from rlbench.tasks.place_shape_in_shape_sorter import PlaceShapeInShapeSorter
from rlbench.tasks.play_jenga import PlayJenga
from rlbench.tasks.plug_charger_in_power_supply import PlugChargerInPowerSupply
from rlbench.tasks.pour_from_cup_to_cup import PourFromCupToCup
from rlbench.tasks.press_switch import PressSwitch
from rlbench.tasks.push_button import PushButton
from rlbench.tasks.push_buttons import PushButtons
from rlbench.tasks.put_all_groceries_in_cupboard import \
    PutAllGroceriesInCupboard
from rlbench.tasks.put_books_on_bookshelf import PutBooksOnBookshelf
from rlbench.tasks.put_bottle_in_fridge import PutBottleInFridge
from rlbench.tasks.put_groceries_in_cupboard import PutGroceriesInCupboard
from rlbench.tasks.put_item_in_drawer import PutItemInDrawer
from rlbench.tasks.put_knife_in_knife_block import PutKnifeInKnifeBlock
from rlbench.tasks.put_knife_on_chopping_board import PutKnifeOnChoppingBoard
from rlbench.tasks.put_money_in_safe import PutMoneyInSafe
from rlbench.tasks.put_plate_in_colored_dish_rack import \
    PutPlateInColoredDishRack
from rlbench.tasks.put_rubbish_in_bin import PutRubbishInBin
from rlbench.tasks.put_shoes_in_box import PutShoesInBox
from rlbench.tasks.put_toilet_roll_on_stand import PutToiletRollOnStand
from rlbench.tasks.put_tray_in_oven import PutTrayInOven
from rlbench.tasks.put_umbrella_in_umbrella_stand import \
    PutUmbrellaInUmbrellaStand
from rlbench.tasks.reach_and_drag import ReachAndDrag
from rlbench.tasks.reach_target import ReachTarget
from rlbench.tasks.remove_cups import RemoveCups
from rlbench.tasks.scoop_with_spatula import ScoopWithSpatula
from rlbench.tasks.screw_nail import ScrewNail
from rlbench.tasks.set_the_table import SetTheTable
from rlbench.tasks.setup_checkers import SetupCheckers
from rlbench.tasks.setup_chess import SetupChess
from rlbench.tasks.slide_block_to_target import SlideBlockToTarget
from rlbench.tasks.slide_cabinet_open import SlideCabinetOpen
from rlbench.tasks.slide_cabinet_open_and_place_cups import \
    SlideCabinetOpenAndPlaceCups
from rlbench.tasks.solve_puzzle import SolvePuzzle
from rlbench.tasks.stack_blocks import StackBlocks
from rlbench.tasks.stack_chairs import StackChairs
from rlbench.tasks.stack_cups import StackCups
from rlbench.tasks.stack_wine import StackWine
from rlbench.tasks.straighten_rope import StraightenRope
from rlbench.tasks.sweep_to_dustpan import SweepToDustpan
from rlbench.tasks.take_cup_out_from_cabinet import TakeCupOutFromCabinet
from rlbench.tasks.take_frame_off_hanger import TakeFrameOffHanger
from rlbench.tasks.take_item_out_of_drawer import TakeItemOutOfDrawer
from rlbench.tasks.take_lid_off_saucepan import TakeLidOffSaucepan
from rlbench.tasks.take_money_out_safe import TakeMoneyOutSafe
from rlbench.tasks.take_off_weighing_scales import TakeOffWeighingScales
from rlbench.tasks.take_plate_off_colored_dish_rack import \
    TakePlateOffColoredDishRack
from rlbench.tasks.take_shoes_out_of_box import TakeShoesOutOfBox
from rlbench.tasks.take_toilet_roll_off_stand import TakeToiletRollOffStand
from rlbench.tasks.take_tray_out_of_oven import TakeTrayOutOfOven
from rlbench.tasks.take_umbrella_out_of_umbrella_stand import \
    TakeUmbrellaOutOfUmbrellaStand
from rlbench.tasks.take_usb_out_of_computer import TakeUsbOutOfComputer
from rlbench.tasks.toilet_seat_down import ToiletSeatDown
from rlbench.tasks.toilet_seat_up import ToiletSeatUp
from rlbench.tasks.turn_oven_on import TurnOvenOn
from rlbench.tasks.turn_tap import TurnTap
from rlbench.tasks.tv_on import TvOn
from rlbench.tasks.unplug_charger import UnplugCharger
from rlbench.tasks.water_plants import WaterPlants
from rlbench.tasks.weighing_scales import WeighingScales
from rlbench.tasks.wipe_desk import WipeDesk

FS10_V1 = {
    'train': [
        ReachTarget,
        CloseBox,
        CloseMicrowave,
        PlugChargerInPowerSupply,
        ToiletSeatDown,
        TakeUmbrellaOutOfUmbrellaStand,
        PutUmbrellaInUmbrellaStand,
        SlideCabinetOpen,
        CloseFridge,
        PickAndLift
    ],
    'test': [
        OpenBox,
        OpenMicrowave,
        UnplugCharger,
        ToiletSeatUp,
        OpenFridge,
    ]
}

FS25_V1 = {
    'train': FS10_V1['train'] + FS10_V1['test'] + [
        TurnTap,
        LightBulbIn,
        BasketballInHoop,
        OpenWindow,
        CloseDoor,
        PushButton,
        PutItemInDrawer,
        OpenDrawer,
        CloseDrawer,
        TurnOvenOn
    ],
    'test': [
        LightBulbOut,
        TvOn,
        OpenOven,
        OpenDoor,
        TakeItemOutOfDrawer
    ]
}

FS50_V1 = {
    'train': FS25_V1['train'] + FS25_V1['test'] + [
        BeatTheBuzz,
        BlockPyramid,
        ChangeClock,
        CloseJar,
        CloseLaptopLid,
        EmptyContainer,
        EmptyDishwasher,
        GetIceFromFridge,
        HangFrameOnHanger,
        InsertOntoSquarePeg,
        PutRubbishInBin,
        PutShoesInBox,
        PutToiletRollOnStand,
        PutTrayInOven,
        ReachAndDrag,
        RemoveCups,
        ScoopWithSpatula,
        SetTheTable,
        SetupCheckers,
        SlideBlockToTarget
    ],
    'test': [
        Hockey,
        InsertUsbInComputer,
        PressSwitch,
        PlayJenga,
        MeatOffGrill,
    ]
}

FS95_V1 = {
    'train': FS50_V1['train'] + FS50_V1['test'] + [
        HitBallWithQueue,
        ScrewNail,
        LampOff,
        LampOn,
        MeatOnGrill,
        MoveHanger,
        OpenJar,
        OpenWineBottle,
        PlaceCups,
        PlaceHangerOnRack,
        PlaceShapeInShapeSorter,
        PutBottleInFridge,
        PutKnifeInKnifeBlock,
        PutMoneyInSafe,
        PutPlateInColoredDishRack,
        SlideCabinetOpenAndPlaceCups,
        StackBlocks,
        StackCups,
        StackWine,
        StraightenRope,
        SweepToDustpan,
        TakeCupOutFromCabinet,
        TakeFrameOffHanger,
        TakeLidOffSaucepan,
        TakeMoneyOutSafe,
        TakeOffWeighingScales,
        TakePlateOffColoredDishRack,
        TakeShoesOutOfBox,
        TakeToiletRollOffStand,
        TakeUsbOutOfComputer,
        WaterPlants,
        WeighingScales,
        WipeDesk,
        ChangeChannel,
        OpenGrill,
        CloseGrill,
        SolvePuzzle,
        PickUpCup,
        PhoneOnBase,
        PourFromCupToCup
    ],
    'test': [
        PutKnifeOnChoppingBoard,
        PutBooksOnBookshelf,
        PushButtons,
        PutGroceriesInCupboard,
        TakeTrayOutOfOven,
    ]
}

MT15_V1 = {
    'train': FS10_V1['train'] + FS10_V1['test']
}


MT30_V1 = {
    'train': FS25_V1['train'] + FS25_V1['test']
}

MT55_V1 = {
    'train': FS50_V1['train'] + FS50_V1['test']
}

MT100_V1 = {
    'train': FS95_V1['train'] + FS95_V1['test']
}
